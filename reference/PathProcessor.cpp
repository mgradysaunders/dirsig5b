#include "dirsig5/internal/core/paths/PathProcessor.h"

#include "dirsig5/internal/core/components/PlanckState.h"
#include "dirsig5/internal/core/components/Problem.h"
#include "dirsig5/internal/core/managers/SingletonManager.h"

namespace d5::core {

namespace PathProcessorGates {

struct Info {
  const SourceManager::LaserPulseWrapper *lpw;
  Node *node;
  double pt0, pt1;
  SceneManager *sm;
  double g0, g1;
  size_t gateCount;
  double gateDelta;
  size_t nWavelengths;
  const SpectralVector *pathWeight;
  double maxGateValue;
  double *buffer; // does not include passive
};

/**
 * A functor that tracks the max (used for convergence testing)
 */
struct PlusMax {
  PlusMax(double &_maxVal) : maxVal(_maxVal) {}

  auto operator()(const double &a, const double &b) const {
    double c = a + b;
    if (c > this->maxVal) this->maxVal = c;
    return c;
  }

  double &maxVal;
};

/**
 * This function is called to temporally render a modulating source
 * into the gate.
 */
bool addSourceToGate(const Source *source, const SpectralVector &weights, void *arg) {
  static thread_local SpectralVector binIrradiance;

  // grab the gate info
  Info &info = *(Info *)(arg);
  Node &node = *info.node;

  // compute width of each gate bin
  double binWidth = (info.g1 - info.g0) / (info.gateCount - 1);

  // loop over the times and render the source
  for (size_t bIndex = 0; bIndex < info.gateCount; bIndex++) {
    // compute the mid time of this time bin
    double binTime = info.g1 + ((bIndex + 0.5) * binWidth);

    // get the source irradiance for this time
    source->getIrradiance(node.sceneID, binTime, node.rayOrigin.cast<double>(), binIrradiance);

    // fold in the weights
    binIrradiance.array() *= weights.array();

    // now add it to the buffer
    std::transform(
      &info.buffer[bIndex * info.nWavelengths], &info.buffer[(bIndex + 1) * info.nWavelengths], binIrradiance.array().data(),
      &info.buffer[bIndex * info.nWavelengths], PlusMax(info.maxGateValue));
  }

  // convert the max value back to radiance
  info.maxGateValue *= binWidth;

  return true;
}

/**
 * This function is called for each surface overalapping beam to look
 * for beam-ray intersections and their contributions
 */
bool addBeamToGate(size_t index, void *arg) {
  Info &info = *(Info *)(arg);
  const SourceManager::LaserPulseWrapper &lpw = *info.lpw;
  static thread_local Vector hit, normal;
  Node &node = *info.node;
  normal = node.normal.cast<double>();
  hit = node.rayOrigin.cast<double>();

  size_t v0i = lpw.tris[index * 3 + 0];
  size_t v1i = lpw.tris[index * 3 + 1];
  size_t v2i = lpw.tris[index * 3 + 2];

  // grab each vertex pair
  const Point &v0a = lpw.points0[v0i];
  const Point &v1a = lpw.points0[v1i];
  const Point &v2a = lpw.points0[v2i];

  static thread_local Point v0, v1, v2;
  const Vector &d0 = lpw.dirs[v0i];
  const Vector &d1 = lpw.dirs[v1i];
  const Vector &d2 = lpw.dirs[v2i];

  // now propagate onto the tangential plane ignore grazing hits
  double d0dotN = d0.dot(normal);
  if (fabs(d0dotN) < 1e-4) return true;
  double d1dotN = d1.dot(normal);
  if (fabs(d1dotN) < 1e-4) return true;
  double d2dotN = d2.dot(normal);
  if (fabs(d2dotN) < 1e-4) return true;

  double dist0 = ((hit - v0a).dot(normal) / (d0dotN));
  double dist1 = ((hit - v1a).dot(normal) / (d1dotN));
  double dist2 = ((hit - v2a).dot(normal) / (d2dotN));

  // check if we're completely obscured
  bool obscured0 = dist0 > lpw.freePath[v0i];
  bool obscured1 = dist1 > lpw.freePath[v1i];
  bool obscured2 = dist2 > lpw.freePath[v2i];
  if (obscured0 && obscured1 && obscured2) {
    return true;
  }

  // calculate tangential plane hit points
  v0 = v0a + dist0 * d0;
  v1 = v1a + dist1 * d1;
  v2 = v2a + dist2 * d2;

  // compute (twice) the triangle area
  double triArea = (v1 - v0).cross(v2 - v0).norm();

  // calculate our barycentric coordinates
  double u = (v1 - hit).cross(v2 - hit).norm() / triArea;

  if (u > 1.f) return true;

  double v = (v0 - hit).cross(v2 - hit).norm() / triArea;

  if (v > 1.f) return true;

  double w = (v0 - hit).cross(v1 - hit).norm() / triArea;

  if (w > 1.f) return true;

  static const double eps = 1e-6;

  if (((u + v + w) - 1.f) > eps) return true;

  triArea /= 2.f;

  // figure out the weighting from focusing/defocusing, r^2 falloff
  // and area projection
  double geometricWeight = lpw.triAreas[index] / triArea;

  // interpolate the pulse energy distribution
  double energyWeight = u * lpw.energyDistribution[v0i] + v * lpw.energyDistribution[v1i] + w * lpw.energyDistribution[v2i];

  // interpolate the shadowing when partially obscured
  double shadowWeight = 1.0 - (u * double(obscured0) + v * double(obscured1) + w * double(obscured2));

  // interpolate the direction
  Vectorf idir = (u * d0 + v * d1 + w * d2).cast<float>();
  idir.normalize();

  // compute the scattering
  static thread_local SpectralVector spectralWeights, spectralVals;
  node.getBSDF(-idir, spectralWeights);

  // incorporate the weight along the path
  spectralWeights *= *info.pathWeight;

  // incorporate all the other weights into a single spectral weight
  spectralWeights *= geometricWeight * energyWeight * shadowWeight;

  // compute the distance traveled from t0
  double dist = u * dist0 + v * dist1 + w * dist2;

  // we need to propagate through the scene so grab the nominal ior
  double ior = info.sm->getRefractiveIndex(node.sceneID);

  // calculate the nominal intersect time of the pulse (the center)
  // in the gate
  double t = info.pt0 + dist / (speedOfLight / ior);

  // figure out where we are within the central bin
  double tc = (t - info.g0);
  double tci = std::floor(tc / info.gateDelta);

  double tc0 = (tci + 0) * info.gateDelta;
  double tc1 = (tci + 1) * info.gateDelta;

  // see if we have pre-generated integrals
  if (!lpw.temporalIntegralsGenerated) {
    static std::mutex mutex;
    std::lock_guard<std::mutex> lock(mutex);

    if (!lpw.temporalIntegralsGenerated) {
      lpw.generateTemporalIntegrals(info.gateDelta);
    }
  }

  assert(!lpw.pulseO.empty());
  assert(!lpw.pulseN.empty());
  assert(!lpw.pulseP.empty());

  int hw = static_cast<int>((lpw.pulseO.size() - 1) / 2);

  if (tc < tc1) {
    double f = 1.0 - (tc - tc0) / (tc1 - tc0);

    for (size_t ii = 0; ii < lpw.pulseO.size(); ii++) {
      int bi = tci + ii - hw + 1;
      if (bi < 0) continue;
      if (bi >= (int)info.gateCount) continue;

      // we want power units so compute the bin average
      double bint = f * lpw.pulseN[ii] + (1.0 - f) * lpw.pulseO[ii];
      spectralVals.array() = lpw.info.pulseEnergy * spectralWeights;
      spectralVals.array() *= bint / info.gateDelta;

      // now add it to the buffer
      std::transform(
        &info.buffer[bi * info.nWavelengths], &info.buffer[(bi + 1) * info.nWavelengths], spectralVals.array().data(),
        &info.buffer[bi * info.nWavelengths], PlusMax(info.maxGateValue));
    }
  } else {
    double tc2 = 1.0 - (tci + 1) * info.gateDelta;
    double f = (tc - tc1) / (tc2 - tc1);

    for (size_t ii = 0; ii < lpw.pulseO.size(); ii++) {
      int bi = tci + ii - hw + 1;
      if (bi < 0) continue;
      if (bi >= (int)info.gateCount) continue;

      // we want power units so compute the bin average
      double bint = f * lpw.pulseO[ii] + (1.0 - f) * lpw.pulseP[ii];
      spectralVals.array() = lpw.info.pulseEnergy * spectralWeights;
      spectralVals.array() *= bint / info.gateDelta;

      // now add it to the buffer
      std::transform(
        &info.buffer[bi * info.nWavelengths], &info.buffer[(bi + 1) * info.nWavelengths], spectralVals.array().data(),
        &info.buffer[bi * info.nWavelengths], PlusMax(info.maxGateValue));
    }
  }

  // convert the max value back to radiance
  info.maxGateValue *= info.gateDelta;

#if _DEBUG
  static std::ofstream out("tris.dat");
  out << v0.transpose() << std::endl;
  out << v1.transpose() << std::endl;
  out << v2.transpose() << std::endl;
  out << v0.transpose() << std::endl;
  out << std::endl << std::endl;

  static std::ofstream hout("hits.dat");
  hout << hit.transpose() << ' ' << spectralWeights[0] << std::endl;

  static std::ofstream aout("areas.dat");
  aout << std::scientific << (hit - Point(0, 0, 2000)).squaredNorm() << ' ' << triArea << ' ' << lpw.triAreas[index] << ' '
       << (triArea / lpw.triAreas[index]) * fabs(normal.dot(lpw.info.pointing)) << std::endl;
#endif
  return true;
}

} // namespace PathProcessorGates

void PathProcessor::processPath(Problem &problem, size_t nWavelengths, size_t refWavelength) {
  // get the globals that we need
  auto &atmManager = MutableSingleton<AtmosphereManager>();
  auto &mediumManager = MutableSingleton<MediumManager>();
  auto &sceneManager = MutableSingleton<SceneManager>();
  auto &sourceManager = MutableSingleton<SourceManager>();
  auto &tempManager = MutableSingleton<TempModelManager>();

  DIRSIG_THROW_IF(problem.buffer == 0, "Null output buffer!");

  // update scratch space
  static thread_local Scratch scratch;
  scratch.update(nWavelengths);

  // use scratch svs avoid allocating/de-allocating data
  SpectralVector &pathL = scratch.sv[0];
  SpectralVector &pathWeight = scratch.sv[1];
  SpectralVector &spectralWeights = scratch.sv[2];
  SpectralVector &spectralVals = scratch.sv[3];
  SpectralVector &mediumL = scratch.sv[4];

  Pointf &viewLoc = scratch.pt[0];
  Vectorf &viewDir = scratch.vc[0];

  // start the pathWeight at 1
  pathWeight.setData(1);

  // and the path radiance at 0
  pathL.setData(0);

  // see how many nodes we are dealing with
  size_t nodeCount = problem.getNodeCount();

  // track whether we'll add the sky
  bool addSky = true;

  // flag to use the ray origin for sky queries
  bool useRayOrigin = true;

  // we still need to check for obscured secondary sources
  apis::Ray &srcRay = scratch.ray;

  // we'll flag any deviants we find along the way
  float deviantWeight = 0.f;

  // local source vector at each node
  Vectorf localSrc;

  // use a fixed epsilon for unobscured path distances
  static const float eps = 1e-3;

  // walk through the path
  size_t nodeIndex = 0;
  float lastViewMu = 1.;
  Node *firstNode = 0;
  bool mediumOnly = true;
  float interfaceTempK = 0.f;
  bool skipDeviantCheck = false;
  size_t segmentIndex = 0;

  for (; nodeIndex < nodeCount; nodeIndex++) DIRSIG_TRY {
      // assume we'll add the sky .. until we know otherwise
      addSky = true;

      // as long as we have nodes we don't use the ray
      useRayOrigin = false;

      Node &node = problem.getNode(nodeIndex);
      if (nodeIndex == 0) firstNode = &node;

      // make sure we don't do anything with the core
      if ((node.opticalPropID == CoreEarthMaterial) && (node.sceneID == EarthSceneID)) {
        addSky = false;
        break;
      }

      // grab the node properties
      OpticalProp *nodeProp = node.opticalProp;

      // construct the viewPoint for the path segment
      viewLoc = node.pathPoint + node.pathDist * node.exitantDirection;

      viewDir = -node.exitantDirection;

      // stuff our path info into a reference-based struct
      mediumL.setData(0.0);
      PathInfo pathInfo(
        viewLoc, viewDir, node.pathDist, node.hitDist, nodeProp, node.normal, node.elapsedTime, node.sceneID, mediumL,
        pathWeight, nodeIndex, segmentIndex, &node.mediumStack);

      TempModel *tempModel = 0;
      if (node.tempModelID != INVALID_ID) {
        tempModel = tempManager.getTempModel(node.sceneID, node.tempModelID);
      }

      // check our medium stack for what we're doing...
      if ((node.mediumStack.size() == 0) || (node.mediumStack.top() == DefaultMedium)) {
        DIRSIG_ERROR_TRACE("applying atmosphere to a path", {
          // predict temperatures
          if ((node.proxySceneID == -1) && (node.hitDist < apis::RayMax)) {
            if (tempModel != 0) {
              tempModel->predictTemperature(node);
            }
          }

          // see if we're along a segment
          if (node.geomID != INVALID_ID) {
            // by default we're in the atmosphere and call directly
            atmManager.applyPath(pathInfo);

            // store information about the path for truth collection
            if (nodeIndex == 0 && problem.tidx.collectPathTruth()) {
              firstNode->pathRadiance += mediumL[refWavelength];
              firstNode->pathTransmission *= pathWeight[refWavelength];
            }
          }

          pathL += mediumL;

          if (nodeIndex > 0 && mediumOnly) {
            mediumOnly = false;
            if (firstNode->temperature == 0) {
              // copy over the first surface temp
              firstNode->temperature = node.temperature;
            }
          }

          // reset the segment index (if any)
          segmentIndex = 0;

          addSky = true;
        })
      } else {
        DIRSIG_ERROR_TRACE("applying medium to a path", {
          // make sure we have a valid path in a user medium
          if (std::isnan(viewLoc[0])) {
            // assume something went wrong
            while (!node.mediumStack.empty() && node.mediumStack.top() != DefaultMedium) node.mediumStack.pop();

            addSky = true;
            break;
          }

          IntID mediumID = node.mediumStack.top();
          Medium *medium = mediumManager.getMedium(mediumID);

          // check for a temperature associated with the property
          // this can be overriden by the model
          if (medium->hasSimpleAbsorption()) {
            if (tempModel != nullptr) {
              tempModel->predictTemperature(node);
              pathInfo.tempK = node.temperature;
            }
          } else {
            // pass in the last surface temp
            pathInfo.tempK = interfaceTempK;
          }

          // apply the medium to the path
          medium->applyPath(pathInfo, lastViewMu);
          pathL += mediumL;
          segmentIndex++;

          // never directly add the sky within a medium
          addSky = false;

          // update path truth
          if (mediumOnly) {
            firstNode->volumePathTempK += pathInfo.pathTempK;
            firstNode->volumePathLength += pathInfo.pathDist;
            firstNode->pathRadiance += node.sampleWeight[refWavelength] * mediumL[refWavelength];
            if (pathInfo.viewTransmission.size() > 0) {
              firstNode->pathTransmission *= pathInfo.viewTransmission[refWavelength];
            }
          }

          // make sure there is a reason to continue
          if (pathWeight.max() <= 1e-20) break;

          // if we just scattered, weight and continue
          if (node.pathDist < node.hitDist) {
            // use any sample weighting from the scattering
            pathWeight *= node.sampleWeight;

            // make sure its worth going on
            double maxWeight = pathWeight.max();
            if ((maxWeight < 1e-20) || (nodeIndex > 0 && maxWeight < problem.pout.convergenceCriteria.threshold)) {
              break;
            }
            continue;
          }
        })
      }

      // see if we found a null material stochastically
      // (but not as part of a path scattering)
      if (node.opticalPropID == NullPropID) {
        // this is a regular null hit .. just add to the distance
        if (nodeIndex < (nodeCount - 1)) {
          // don't add to a miss
          if (problem.getNode(nodeIndex + 1).geomID != INVALID_ID) {
            problem.getNode(nodeIndex + 1).hitDist += node.hitDist;
          }

          continue;
        } else {
          break;
        }
      }

      // make sure we have something to work with..
      if (nodeProp == 0) {
        // check for hitting the sky directly (only if we can still hit
        // it)
        if (addSky) {
          if (node.geomID == INVALID_ID && node.pathDist == node.hitDist) {
            useRayOrigin = nodeIndex == 0;
            break;
          }
        }

        if (nodeIndex == 0) skipDeviantCheck = true;

        continue;
      }

      // if this is a transition surface, do nothing else
      if (nodeProp->isMediumTransition()) {
        addSky = false;

        if (nodeIndex == 0) skipDeviantCheck = true;

        continue;
      }

      // check for self-emission (also zero if out of spectral range)
      float viewMu = fabs(node.exitantDirection.dot(node.normal));
      if (node.temperature > 0) {
        if (node.opticalProp && !node.opticalProp->hasTransmittance()) {
          node.getBEDF(spectralWeights);
          pathL += PlanckState::getInstance()->getBlackbodySurfaceEmittance(node.temperature, viewMu).array() *
                   spectralWeights.array() * pathWeight.array() / viewMu;
        }
        interfaceTempK = node.temperature;
      }
      lastViewMu = viewMu;

      // add in exoatmospheric source contributions
      for (size_t si = 0; si < node.exoSourceInfo.size(); si++) {
        DIRSIG_ERROR_TRACE("processing exoatmospheric source", {
          if (!node.exoSourceInfo[si].hitSource) {
            continue;
          }

          float srcMu = 1.f;
          const Vectorf &globalSrc = atmManager.getSceneExoSourceVector(node.sceneID, si, srcMu);

          localSrc = node.spaceTransform * globalSrc;

          {
            // find the scattered contribution
            node.getBSDF(globalSrc, spectralWeights, srcMu);

            bool isDelta =
              node.localExitant[2] * localSrc[2] >= 0 ? nodeProp->isDeltaReflectance() : nodeProp->isDeltaTransmittance();

            /**
             * Define a high BSDF value by exceeding 1.0/pi (100% R)
             */
            if (spectralWeights[refWavelength] > M_1_PI && !isDelta) {
              auto rot = makeRotationf(Vectorf(0, 0, 1), globalSrc);
              size_t nSunSamples = 10;
              srcMu = std::max(srcMu, -1.0f);
              srcMu = std::min(srcMu, +1.0f);
              float srcTan = std::sqrt(1 - srcMu * srcMu) / srcMu;
              SpectralVector sampleBSDF;
              for (size_t ii = 1; ii < nSunSamples; ii++) {
                // average a sampling of the solar disk
                float xi1 = canonical(problem.ray->randEngine);
                float xi2 = canonical(problem.ray->randEngine);

                // we'll model the sun as a disk 1m away
                float sqrtx = srcTan * std::sqrt(xi1);
                float theta = 2 * M_PI * xi2;
                Vectorf sampleDir = rot * Vectorf(sqrtx * std::cos(theta), sqrtx * std::sin(theta), 1.f);
                sampleDir.normalize();
                node.getBSDF(sampleDir, sampleBSDF, srcMu);
                spectralWeights.array() += sampleBSDF.array();
              }

              spectralWeights.array() /= nSunSamples;
            }
          }

          // get the solar irradiance (includes primary transmission)
          atmManager.getSceneExoSourceIrradiance(node.sceneID, node.pathPoint, si, spectralVals);

          // check if we passed into the atmosphere medium
          if ((node.mediumStack.size() > 0 && (node.mediumStack.top() != DefaultMedium && node.hitBackside))) {
            // handle n^2 law for radiance
            {
              IntID mediumID = node.mediumStack.top();
              Medium *medium = mediumManager.getMedium(mediumID);

              // apply the medium to the path
              static thread_local SpectralVector refractiveIndex(spectralWeights.size());
              medium->getRefractiveIndex(node.rayOrigin, node.elapsedTime, true, refractiveIndex);

              spectralWeights *= refractiveIndex * refractiveIndex;
            }
          }

          node.exoSourceInfo[si].exoIrradiance = spectralVals[refWavelength];

          spectralVals.array() *= node.exoSourceInfo[si].NdotS * node.exoSourceInfo[si].weighting.array() * pathWeight.array();

          spectralVals *= spectralWeights;

          if (nodeIndex == 0 || skipDeviantCheck) {
            // always add the first node
            pathL += spectralVals;
            skipDeviantCheck = false;
          } else {

            // see if we have high scattering > 50% lambertian
            // and we're not following a mirror
            static const float deviantThreshold = 0.5 / M_PI;
            if (spectralWeights[refWavelength] > deviantThreshold && pathWeight[refWavelength] < 0.5) {
              deviantWeight += std::max(0.1, pathWeight[refWavelength] * (1.f - spectralWeights[refWavelength]));
            }

            pathL += spectralVals;
          }
        })
      }

      size_t sourceCount = sourceManager.getSourceCount();

      // walk through the available sources, optimizing as we go
      if (sourceCount > 0)
        DIRSIG_ERROR_TRACE("processing secondary sources", {
          double sourceMultiplier = 1.0;
          double multiplierScale = 1.0;

          // only bother optimizing if we have a lot of sources on
          // first node
          if (sourceCount > 12 && nodeIndex == 0 && problem.sampleCount < problem.pout.convergenceCriteria.minimumSamples) {
            sourceMultiplier = sourceCount / double(problem.pout.convergenceCriteria.minimumSamples);
            if (sourceMultiplier < 1) sourceMultiplier = 1.;
            multiplierScale = double(sourceCount) / sourceMultiplier;
          } else {
            sourceMultiplier = sourceCount;
            multiplierScale = 1.0;
          }

          // we don't know where the source contributions are so just
          // start with sampling all of them
          std::uniform_int_distribution<int> randi(0, sourceCount - 1);

          // grab the radiance threshold from the problem
          double threshold = problem.getRadianceThreshold();

          Vectorf d;
          Source *source;
          bool sourceFound;
          size_t randIndex;
          for (size_t si = 0; si < size_t(ceil(sourceMultiplier)); si++) {
            sourceFound = false;

            if (multiplierScale != 1.0) {
              if (nodeIndex == 0) {
                if (problem.sourceIndex < (sourceCount - 1)) {
                  randIndex = problem.sourceIndex++;
                } else {
                  problem.sourceIndex = 0;
                  randIndex = 0;
                }
              } else {
                randIndex = randi(problem.ray->randEngine);
              }
            } else {
              randIndex = si;
            }

            source = sourceManager.getSource(randIndex);

            Pointf so = source->spatialSample(node.sceneID, node.rayOrigin, static_cast<float>(node.elapsedTime));

            // get the unnormalized direction
            d = (so - node.pathPoint);

            // and the squared distance to the source
            float d2 = d.squaredNorm();

            // check if there is a remote possiblity this will be
            // useful
            float power = source->getReferencePower();

            float dist = sqrtf(d2);
            d /= dist;

            float NdotS = d.dot(node.normal);

            // do a basic check on the side
            if ((!nodeProp->hasTransmittance() && NdotS < 0) || (!nodeProp->hasReflectance() && NdotS > 0)) {
              continue;
            }

            // skip if we're below threshold and not facing the
            // light
            if ((power / (4.f * M_PI * d2)) < threshold) {
              // grab the pointing direction (assumed peak)
              Vectorf pointing = source->getDirection(node.elapsedTime).cast<float>();

              // ... direct cutoff (account for focused beams)
              if (pointing.dot(d) > -0.999) {
                continue;
              }
            }

            // fetch the source irradiance
            SpectralVector sourceIrradiance;
            source->getIrradiance(node.sceneID, node.elapsedTime, node.rayOrigin.cast<double>(), sourceIrradiance);

            // incorporate weightings (geometric and otherwise)
            spectralVals.array() = sourceIrradiance.array();
            spectralVals *= fabs(NdotS) * pathWeight * multiplierScale;

            // now check against the actual values (without surface)
            if (spectralVals.max() < threshold) {
              continue;
            }

            // now construct a ray to check for obscuration
            srcRay.initLocalSceneRay(node.sceneID);
            srcRay.org = node.rayOrigin;
            srcRay.dir = d;

            // only go as far as the source sample with some padding
            srcRay.tfar = dist - eps;
            srcRay.tnear += eps;

            // update the time and reset geometry
            srcRay.elapsedTime = node.elapsedTime;
            srcRay.geomID = INVALID_ID;

            // flag high priority if direct lighting calculation
            srcRay.highPriority = nodeIndex <= 1 ? true : false;

            // check for visibility (no trans currently)
            sceneManager.intersect(srcRay);

            if (srcRay.geomID == unsigned(INVALID_ID)) {
              sourceFound = true;
            } else {
              // propagate through any null materials
              sceneManager.setSceneMatID(srcRay);

              if (srcRay.matID < 0) {
                // make sure we don't keep intersecting ourself
                unsigned nullGeomID = srcRay.geomID;
                unsigned nullPrimID = srcRay.primID;

                size_t bailCount = 0;
                while (srcRay.matID < 0) {
                  srcRay.tnear = srcRay.tfar;
                  srcRay.tfar = dist;

                  do {
                    srcRay.tnear += eps;
                    srcRay.tfar -= eps;
                    if (srcRay.tfar <= srcRay.tnear) {
                      break;
                    }
                    srcRay.geomID = -1;
                    srcRay.skip = false;
                    sceneManager.intersect(srcRay);

                  } while (srcRay.geomID == nullGeomID && srcRay.primID == nullPrimID);

                  if (srcRay.geomID == unsigned(INVALID_ID)) {
                    break;
                  }

                  // get the scene material
                  sceneManager.setSceneMatID(srcRay);

                  // make sure we don't get stuck in a loop
                  if (bailCount++ >= 10) {
                    break;
                  }
                }

                if (srcRay.geomID == unsigned(INVALID_ID)) {
                  sourceFound = true;
                }
              }
            }

            // check if we had a clear path to the source
            if (sourceFound) {
              node.getBSDF(Vectorf(srcRay.dir[0], srcRay.dir[1], srcRay.dir[2]), spectralWeights);
              // weight the contribution by the optical property
              spectralVals.array() *= spectralWeights.array();

              // always directly add on the first node
              if (nodeIndex == 0) {
                // increase the number of sources
                problem.sourcesUsed += multiplierScale;

                // if we want gated radiance, temporally render
                // the
                if (problem.gateCount > 0) {
                  static thread_local PathProcessorGates::Info info;
                  info.g0 = problem.gateOffset - 0.5 * problem.gateDelta;
                  info.g1 = problem.gateOffset + (problem.gateCount + 0.5) * problem.gateDelta;
                  info.gateCount = problem.gateCount;
                  info.gateDelta = problem.gateDelta;
                  info.buffer = &problem.buffer[nWavelengths];
                  info.nWavelengths = nWavelengths;
                  info.maxGateValue = 0.0;
                  info.node = &node;
                  info.sm = &sceneManager;
                  info.pathWeight = &pathWeight;

                  // pull the source irradiance to get the
                  // weights
                  SpectralVector srcWeights;
                  srcWeights.array() = spectralVals.array() / sourceIrradiance.array();

                  // add this source to the gate
                  PathProcessorGates::addSourceToGate(source, srcWeights, (void *)&info);

                  // check if we should update the max gate
                  // value
                  if (info.maxGateValue > problem.maxGateValue) {
                    problem.maxGateValue = info.maxGateValue;
                  }
                } else {
                  // otherwise, just store the temporal
                  // average
                  pathL += spectralVals;
                }
              } else {
                // if latter node exceeds the first, flag as
                // deviant
                if (pathL[refWavelength] > 1e-9 && (pathL - spectralVals).minCoeff() < 0) {
                  deviantWeight += 1.f;
                }

                pathL += spectralVals;
              }
            } else {
              continue;
            }
          }
        });

      // if our problem requests gates, set
      if (problem.gateCount > 0) {
        // build an object we can send to the search processor
        static thread_local PathProcessorGates::Info info;

        // compute the gate bounds
        info.g0 = problem.gateOffset - 0.5 * problem.gateDelta;
        info.g1 = problem.gateOffset + (problem.gateCount + 0.5) * problem.gateDelta;
        info.gateCount = problem.gateCount;
        info.gateDelta = problem.gateDelta;
        info.buffer = &problem.buffer[nWavelengths];
        info.nWavelengths = nWavelengths;
        info.maxGateValue = 0.0;
        static thread_local std::vector<double> minR(2), maxR(2);

        auto iter = sourceManager.beginActive();
        for (; iter != sourceManager.endActive(); iter++) {
          // grab the (sample fixed) times for this pulse
          info.pt0 = iter->t0 - node.elapsedTime;
          info.pt1 = iter->t1 - node.elapsedTime;

          // don't bother if its impossible to hit the gate
          if (info.pt1 < info.g0) continue;
          if (info.pt0 > info.g1) continue;

          // find all possible beams that could intersect our hit
          // point
          static thread_local Point hitPrj;
          hitPrj = node.rayOrigin.cast<double>();

          // project the hit point into the 2D R-Tree space
          hitPrj = hitPrj - iter->info.pointing.dot(hitPrj - iter->prjOrigin) * iter->info.pointing;

          // align to the chosen axes and contruct the search point
          hitPrj -= iter->prjOrigin;
          minR[0] = iter->uVec.dot(hitPrj) - 1e-20;
          minR[1] = iter->vVec.dot(hitPrj) - 1e-20;
          maxR[0] = minR[0] + 1e-20;
          maxR[1] = minR[1] + 1e-20;

          info.lpw = &*iter;
          info.node = &node;
          info.sm = &sceneManager;
          info.pathWeight = &pathWeight;

          // add all intersecting beams
          iter->searchTree->Search(minR.data(), maxR.data(), PathProcessorGates::addBeamToGate, (void *)&info);
        }

        // check if we should update the max gate value
        if (info.maxGateValue > problem.maxGateValue) {
          problem.maxGateValue = info.maxGateValue;
        }
      }

      // if this was a mixed null account for weight
      pathWeight *= node.sampleWeight;

      // make sure its worth going on
      double maxWeight = pathWeight.max();
      if ((maxWeight < 1e-20) || (nodeIndex > 0 && maxWeight < problem.pout.convergenceCriteria.threshold)) {
        break;
      }
    }
  DIRSIG_CATCH(error, { throw error.withTraceMessage("walking a node path at index " + std::to_string(nodeIndex)); })

  // make sure we actually missed the scene instead of maxing out
  if (nodeIndex >= problem.getMaxPathLength()) addSky = false;

  // if we are adding the sky get the appropriate radiance
  if (addSky)
    DIRSIG_ERROR_TRACE("adding the sky to a path", {
      // check if we're directly viewing the sky
      if (useRayOrigin || nodeIndex == 0) {
        // check for exoatmospheric paths
        if (problem.ray->coordinates == apis::DoubleEarth) {
          // evaluate the point of closest approach along the ray
          double t = 0;

          // unsquash the ellipsoid via the ray first/
          Point so = problem.ray->ecefOrigin;
          so[2] /= EARTH_ECCENTRICITY;

          Vector sd = problem.ray->ecefDirection;
          sd[2] /= EARTH_ECCENTRICITY;
          sd.normalize();

          t = -(sd[0] * so[0] + sd[1] * so[1] + sd[2] * so[2]) / (sd[0] * sd[0] + sd[1] * sd[1] + sd[2] * sd[2]);

          // account for non-infinite rays
          t = std::min((double)problem.ray->tfar, t);

          // evaluate the position at closest approach
          so += t * sd;

          // we'l check against a 100km altitude limit
          double atmLimit = WGS84EquatorialRadius + 100e3;

          // only worry about the atmosphere if pass through
          if (so.squaredNorm() > atmLimit * atmLimit) {
            // \todo construct an arbitrary atm path
            addSky = false;
          }
          // for now turn off paths that start outside the atm
          else if (problem.ray->ecefOrigin.squaredNorm() > atmLimit * atmLimit) {
            addSky = false;
          }
        }

        if (addSky) {
          viewLoc[0] = problem.ray->org[0];
          viewLoc[1] = problem.ray->org[1];
          viewLoc[2] = problem.ray->org[2];

          viewDir[0] = problem.ray->dir[0];
          viewDir[1] = problem.ray->dir[1];
          viewDir[2] = problem.ray->dir[2];

          // create a path to the sky
          PathInfo pathInfo(
            viewLoc, viewDir, std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), 0, scratch.vc[0],
            problem.ray->elapsedTime, problem.ray->sceneID, pathL, pathWeight, nodeIndex, 0, 0);

          atmManager.applyPath(pathInfo);
        }
      } else {
        Node &lastNode = problem.getNode(nodeIndex - 1);
        PathInfo pathInfo(
          lastNode.rayOrigin, lastNode.rayDirection, std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), 0,
          lastNode.normal, lastNode.elapsedTime, lastNode.sceneID, pathL, pathWeight, nodeIndex, 0, &lastNode.mediumStack);

        atmManager.applyPath(pathInfo);
      }
    });

  // add the result to the problem
  problem.addPathRadiance(pathL, deviantWeight);
}

} // namespace d5::core
