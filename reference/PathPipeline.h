#pragma once

#include "NodeGenerator.h"
#include "PathProcessor.h"
#include "dirsig5/common/Constants.h"
#include "dirsig5/internal/core/components/Problem.h"
#include "dirsig5/internal/core/components/TruthIndexes.h"
#include "dirsig5/internal/core/managers/SingletonManager.h"

#pragma push_macro("emit")
#undef emit
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#pragma pop_macro("emit")

namespace d5::core {

/**
 * The path pipeline takes a vector of problems and steps through the
 * component processes necessary to get to a final radiance solution
 * for each problem.
 */
class PathPipeline {
public:
  PathPipeline() {
    MutableSingleton<StateManager>().registerSpectralStateUpdates([this](const State *state) { updateSpectralState(state); });
  }

  static void updateProxies();

  void setGlobalSeed(unsigned seed) { globalSeed = seed; }

  void resize(std::vector<Problem *> &problems) {
    size_t numProblems = problems.size();
    if (rays.size() < numProblems) {
      // size_t offset = 0;
      // if (rays.size() > 0) {
      //  offset = rays.size() - 1;
      //}
      rays.clear();
      rays.reserve(numProblems);
      raysSource.clear();
      raysSource.reserve(numProblems);
      embreeCores.resize(numProblems);
      embreeCoresSource.resize(numProblems);
      for (size_t ii = 0; ii < numProblems; ii++) {
        rays.emplace_back(&embreeCores[ii]);
        raysSource.emplace_back(&embreeCoresSource[ii]);
        problems[ii]->ray = &rays[ii];
        problems[ii]->exoSourceRay = &raysSource[ii];
      }
    }
  }

  void updateSpectralState(const State *state) {
    DIRSIG_THROW_IF(state == nullptr, "Provided state object is invalid!");
    const auto &spectralState = state->spectralStates->at(state->spectralStateIndex);
    stateWavelengthCount = spectralState.wavelengthSamples.size();
    stateWavelength = spectralState.referenceWavelengthIndex;
  }

  /**
   * Run a vector of problems through the pipeline. Buffers associated
   * with the problems will be filled by the end of the run. Note that
   * a run is performed for a single internal state.
   *
   * The maximum samples argument represents the maximum number of
   * samples across the vector of problems. Individual problem
   * convergence criteria are checked independently for early completion.
   */
  void run(std::vector<Problem *> &problems, size_t maxSamples, size_t maxPathLength);

  static size_t stateWavelengthCount;
  static size_t stateWavelength;

  std::vector<apis::EmbreeRay> embreeCores;
  std::vector<apis::EmbreeRay> embreeCoresSource;
  std::vector<apis::Ray> rays;
  std::vector<apis::Ray> raysSource;

  unsigned globalSeed = 0;
  unsigned threadBlockSize = 8;
};

inline void initPath(unsigned seed, Problem *problem) try {
  // check whether we skip this problem
  if (problem->pout.skipFlag || problem->complete) return;

  // get a random ray for this problem from the callback
  float elapsedTime = 0;
  problem->currentPathWeight = 1;
  problem->ray->randEngine.seed(seed);
  if (!problem->pout.callback(
        problem->ray->randEngine, problem->pin.taskIndex, problem->pin.captureIndex, problem->pin.problemIndex,
        problem->pin.spectralStateIndex, problem->pout.data, problem->po, problem->pd, elapsedTime,
        problem->currentPathWeight)) {
    problem->ray->skip = true;
    problem->pathComplete = true;
    return;
  }

  if (problem->pout.sceneID >= 0) {
    // remap the scene ID to the internal system
    IntID sceneID = MutableSingleton<SceneManager>().getUserSceneID(problem->pout.sceneID);
    problem->ray->initGlobalSceneRay(sceneID);
  } else {
    problem->ray->initGenericRay();
    problem->ray->coordinates = apis::DoubleEarth;
    problem->ray->sceneID = INVALID_ID;
  }

  // set the time since the start of the capture
  problem->ray->elapsedTime = elapsedTime;
  problem->ray->gOrigin[0] = problem->po[0];
  problem->ray->gOrigin[1] = problem->po[1];
  problem->ray->gOrigin[2] = problem->po[2];
  problem->ray->gDirection[0] = problem->pd[0];
  problem->ray->gDirection[1] = problem->pd[1];
  problem->ray->gDirection[2] = problem->pd[2];

  DIRSIG_THROW_IF(!std::isfinite(problem->ray->elapsedTime), "Ray sampler callback returned a non-finite ray time!");
  DIRSIG_THROW_IF(!problem->ray->gOrigin.allFinite(), "Ray sampler callback returned a non-finite ray origin!");
  DIRSIG_THROW_IF(!problem->ray->gDirection.allFinite(), "Ray sampler callback returned a non-finite ray direction!");

  problem->ray->gDirection.normalize();
  problem->initializePath();

  // check whether we're inside a medium
  auto &mm = MutableSingleton<MediumManager>();
  size_t mediumCount = mm.getMediumCount();

  if (mediumCount > 0) {
    for (size_t ii = 1; ii < mediumCount + 1; ii++) {
      Medium *m = mm.getMedium(ii);
      if (m->rayInside(*problem->ray)) {
        problem->mediumStack.push(ii);
        problem->ray->curMedium = m->getID();
        break;
      }
    }
  }

  // update any sensor leaving truth
  problem->tidx.sensorUpdate(*problem);
} catch (std::exception &e) {
  static std::mutex mutex;
  mutex.lock();
  std::cerr << "\nError while initializing a path:\n";
  std::cerr << e.what() << std::endl;
  abort();
}

inline void traceExoSourceRays(Problem *problem) try {
  if (problem->pout.skipFlag || problem->complete || problem->pathComplete) return;

  // grab our node
  Node &node = problem->currentNode();

  // see if we need to bother with exo sources at all
  if (!node.addExoSources) return;

  auto &atmManager = MutableSingleton<AtmosphereManager>();
  auto &opticalPropManager = MutableSingleton<OpticalPropManager>();
  auto &sceneManager = MutableSingleton<SceneManager>();

  // see how many exo sources we have to trace
  size_t nExoSources = atmManager.getExoSourceCount();

  // add each exo source as needed
  node.exoSourceInfo.resize(nExoSources);

  for (size_t si = 0; si < nExoSources; si++) {
    node.exoSourceInfo[si].weighting.resize(PathPipeline::stateWavelengthCount);
    node.exoSourceInfo[si].hitSource = false;
    node.exoSourceInfo[si].weighting.setData(1.0);

    // grab the direction
    float srcMu = 1.f;
    Vectorf srcVector = atmManager.getSceneExoSourceVector(node.sceneID, si, srcMu);

    // cosine effect is always based on the center vector
    node.exoSourceInfo[si].NdotS = srcVector.dot(node.normal);

    // sample the source if we have a roughly >0.2 deg source
    if (srcMu < 0.999994) {
      // sample disk at zenith then rotate
      static thread_local std::vector<Matrixf> rot(nExoSources, Matrixf::Identity());
      static thread_local std::vector<Vectorf> cachedSrcVector(nExoSources, Vectorf(0, 0, 1));
      if (cachedSrcVector[si] != srcVector) {
        cachedSrcVector[si] = srcVector;
        rot[si] = makeRotationf(Vectorf(0, 0, 1), srcVector);
      }

      float srcTan = std::sqrt(1 - srcMu * srcMu) / srcMu;
      float xi1 = canonical(problem->ray->randEngine);
      float xi2 = canonical(problem->ray->randEngine);

      // we'll model the source as a disk 1m away
      float sqrtx = srcTan * sqrtf(xi1);

      float theta = TwoPI * xi2;
      srcVector = rot[si] * Vectorf(sqrtx * cos(theta), sqrtx * sin(theta), 1.f);

      srcVector.normalize();
    }

    apis::Ray *sourceRay = problem->exoSourceRay;

    sourceRay->skip = true;

    bool hasReflectance = false;
    bool hasTransmittance = false;

    // make sure the source ray has the current medium
    // (participating media is handled separately)
    if (!problem->mediumStack.empty()) {
      sourceRay->curMedium = problem->mediumStack.top();
    }

    if (node.mixedProps.size() > 1) {
      for (size_t ii = 0; ii < node.mixedProps.size(); ii++) {
        if (node.mixedProps[ii].second == INVALID_ID) {
          continue;
        }

        OpticalProp *props = opticalPropManager.getOpticalProp(node.sceneID, node.mixedProps[ii].second);

        if (props != 0) {
          if (props->hasReflectance()) hasReflectance = true;
          if (props->hasTransmittance()) hasTransmittance = true;
        }
      }
    } else if (node.opticalProp != 0) {
      hasReflectance = node.opticalProp->hasReflectance();
      hasTransmittance = node.opticalProp->hasTransmittance();
    }

    if (hasReflectance) {
      // only compute the source for node.exitantDirectionrays that are in front
      if (node.exoSourceInfo[si].NdotS > 0.f) {
        Vectorf offset = node.Ng;
        if (offset.dot(srcVector) < 0) offset *= -1.f;
        sourceRay->copy(*problem->ray);
        sourceRay->org = node.pathPoint + SURFACE_EPS * offset;
        sourceRay->dir = srcVector;
        sourceRay->sceneID = node.sceneID;
        sourceRay->elapsedTime = node.elapsedTime;
        sourceRay->skip = false;
      }
    }

    // only check the other side if we haven't found anything yet
    if (sourceRay->skip) {
      if (hasTransmittance) {
        // only compute the src for rays in back of the surface
        if (node.exoSourceInfo[si].NdotS < 0.f) {
          node.exoSourceInfo[si].NdotS *= -1.f;
          sourceRay->initLocalSceneRay(node.sceneID);
          Vectorf offset = node.Ng;
          if (offset.dot(srcVector) < 0) offset *= -1.f;
          sourceRay->org = node.pathPoint - SURFACE_EPS * offset;
          sourceRay->dir = srcVector;
          sourceRay->sceneID = node.sceneID;
          sourceRay->elapsedTime = node.elapsedTime;
          sourceRay->skip = false;
        }
      }
    }

    // still have nothing? move on
    if (sourceRay->skip) {
      continue;
    }

    node.exoSourceInfo[si].hitSource = sceneManager.traceShadowRay(*sourceRay, node.exoSourceInfo[si].weighting);
  }

} catch (std::exception &e) {
  static std::mutex mutex;
  mutex.lock();
  std::cerr << "\nError while tracing a source ray:\n";
  std::cerr << e.what() << std::endl;
  abort();
}

class tbb_generatePath {
  Problem **problems;
  size_t globalSeed;
  size_t maxPathLength;
  size_t maxSamples;

public:
  void operator()(const tbb::blocked_range<size_t> &r) const {
#ifdef _DEBUG
    static size_t totalNodeCount = 0;
#endif
    auto &sm = MutableSingleton<SceneManager>();
    for (size_t ii = r.begin(); ii != r.end(); ++ii) {
      auto *problem = problems[ii];
      problem->initialize(PathPipeline::stateWavelengthCount);
      RandEngine randEngine(problem->seed, globalSeed);
      size_t problemMaxNodes = problem->pout.convergenceCriteria.maximumNodes;
      for (size_t ss = 0; ss < maxSamples; ++ss) {
#ifdef _DEBUG
        size_t pathNodeCount = 0;
#endif
        if (problem->pout.skipFlag or problem->complete) break;
        initPath(randEngine(), problem);
        if (problem->pathComplete) continue;

#ifdef _DEBUG
        std::cout << "v " << problem->ray->gOrigin[0] << " " << problem->ray->gOrigin[1] << " " << problem->ray->gOrigin[2]
                  << "\n";
        totalNodeCount += 1;
        pathNodeCount += 1;
#endif
        problem->ray->highPriority = true;
        for (size_t jj = 0; jj < problemMaxNodes; jj++) {
          if (problem->pout.skipFlag or problem->complete or problem->pathComplete) break;

          sm.intersect(*problem->ray);
          if (jj == 0)
            // update any scene arrival truth
            problem->tidx.sceneUpdate(*problem);

          // Generate node
          if (not problem->pout.skipFlag and not problem->complete and not problem->pathComplete) {
            Node &node = problem->addNode();
            if (NodeGenerator::generateNode(
                  *problem, node, PathPipeline::stateWavelengthCount, PathPipeline::stateWavelength)) {
#ifdef _DEBUG
              std::cout << "v " << node.pathPoint[0] << " " << node.pathPoint[1] << " " << node.pathPoint[2] << "\n";
              totalNodeCount += 1;
              pathNodeCount += 1;
#endif
              IntID sceneID = node.sceneID;
              // look for hitting a proxy, if true, reset and remove
              if (problem->ray->proxyParent != -1) {
                sceneID = problem->ray->proxyParent;
                problem->ray->proxyParent = -1;
              }
              // construct a new ray if we continue
              problem->ray->initLocalSceneRay(sceneID);
              problem->ray->org[0] = node.rayOrigin[0];
              problem->ray->org[1] = node.rayOrigin[1];
              problem->ray->org[2] = node.rayOrigin[2];
              problem->ray->dir[0] = node.rayDirection[0];
              problem->ray->dir[1] = node.rayDirection[1];
              problem->ray->dir[2] = node.rayDirection[2];
              problem->ray->elapsedTime = node.elapsedTime;
              if (not problem->mediumStack.empty()) problem->ray->curMedium = problem->mediumStack.top();
            } else
              problem->pathComplete = true;
            if (problem->pout.skipFlag or problem->complete or problem->pathComplete) problem->ray->skip = true;
          }
          traceExoSourceRays(problem);
          problem->ray->highPriority = false;
        }

#ifdef _DEBUG
        for (size_t ni = 0; ni < pathNodeCount - 1; ni++) {
          std::cout << "l " << totalNodeCount - pathNodeCount + ni + 1 << " " << totalNodeCount - pathNodeCount + ni + 2
                    << std::endl;
        }
#endif

        if (
          not problem->pout.skipFlag and //
          not problem->complete) {
          PathProcessor::processPath(*problem, PathPipeline::stateWavelengthCount, PathPipeline::stateWavelength);
          if (int(problem->getNodeCount()) >= problem->pout.convergenceCriteria.maximumNodes) problem->pathComplete = true;
          problem->checkConvergence(PathPipeline::stateWavelength);
        }
        if (
          problem->pout.skipFlag or //
          problem->complete)
          break;
      }
      // make sure we're really complete
      if (not problem->complete) {
        problem->complete = true;
        problem->checkConvergence(PathPipeline::stateWavelength);
      }
    }
  }
  explicit tbb_generatePath(Problem *_problems[], size_t _globalSeed, size_t _maxPathLength, size_t _maxSamples)
    : problems(_problems), globalSeed(_globalSeed), maxPathLength(_maxPathLength), maxSamples(_maxSamples) {}
};

/**
 * Run a vector of problems through the pipeline. Buffers associated
 * with the problems will be filled by the end of the run. Note that
 * a run is performed for a single internal state.
 *
 * The maximum samples argument represents the maximum number of
 * samples across the vector of problems. Individual problem
 * convergence criteria are checked independently for early completion.
 */
inline void PathPipeline::run(std::vector<Problem *> &problems, size_t maxSamples, size_t maxPathLength) {
  // make sure we have something to work with
  if (problems.size() == 0) return;

  tbb::parallel_for(
    tbb::blocked_range<size_t>(0, problems.size(), threadBlockSize),
    tbb_generatePath(problems.data(), globalSeed, maxPathLength, maxSamples));
}

} // namespace d5::core
