#include "dirsig5/internal/core/paths/NodeGenerator.h"

#include <unordered_map>

#include "dirsig5/common/Constants.h"
#include "dirsig5/common/Geometry.h"
#include "dirsig5/internal/core/components/PathInfo.h"
#include "dirsig5/internal/core/components/Problem.h"
#include "dirsig5/internal/core/managers/SingletonManager.h"

namespace d5::core {

void NodeGenerator::handleMediumTransition(Problem &problem, Node &node, size_t nWavelengths) {
  // see if we entered a new medium
  IntID backsideMedium = INVALID_ID;
  static thread_local SpectralVector ior;
  ior.resize(nWavelengths);

  if (node.opticalProp->hasBacksideMedium(backsideMedium)) {
    if (node.hitBackside) {
      // exiting out of a medium -- pop the stack
      if (!problem.mediumStack.empty()) {
        // only pop if we match mediums
        if (problem.mediumStack.top() == backsideMedium) {
          // adjust the spectral weighting with refractive index
          MutableSingleton<MediumManager>()
            .getMedium(backsideMedium)
            ->getRefractiveIndex(node.pathPoint, node.elapsedTime, true, ior);

          problem.mediumStack.pop();

          node.sampleWeight.array() *= (ior.array() * ior.array());

          // we never add the sun when first exiting
          node.addExoSources = false;
        }
      }
    } else {
      // entering a medium -- push the stack
      problem.mediumStack.push(backsideMedium);

      if (backsideMedium != DefaultMedium) {
        // adjust the spectral weighting with refractive index
        MutableSingleton<MediumManager>()
          .getMedium(backsideMedium)
          ->getRefractiveIndex(node.pathPoint, node.elapsedTime, true, ior);

        node.sampleWeight /= (ior.array() * ior.array());
      }
    }
  }
}

bool NodeGenerator::generateNode(Problem &problem, Node &node, size_t nWavelengths, size_t refWavelength) {
  // start by assuming that the node is visible and 0K
  node.clear();

  // copy over any medium stack info
  node.mediumStack = problem.mediumStack;

  // grab the ray and the scene
  apis::Ray &ray = *problem.ray;
  if (ray.proxyParent != INVALID_ID) {
    node.sceneID = ray.proxyParent;
    node.proxySceneID = ray.sceneID;
  } else {
    node.sceneID = ray.sceneID;
    node.proxySceneID = INVALID_ID;
  }

  node.geomID = ray.geomID;
  node.primID = ray.primID;
  node.instID = ray.instID;
  node.seed = ray.randEngine();

  // calculate how far we traveled to the nearest hit, including propagation
  node.hitDist = ray.propagationDistance + ray.tfar;

  // check for hitting the core -- if we did, stop here
  if (node.sceneID == EarthSceneID) {
    node.opticalPropID = ray.matID;
    if (ray.matID == CoreEarthMaterial) {
      node.stopPath = true;
      return false;
    }
  }

  if (node.sceneID == EarthSceneID)
    node.exitantDirection = -1.f * ray.ecefDirection.cast<float>();
  else
    // track the exitant (view) direction
    node.exitantDirection = -1.f * Vectorf(ray.dir[0], ray.dir[1], ray.dir[2]);

  // calculate how far we traveled along the path (could be scattered)
  auto &mediumManager = MutableSingleton<MediumManager>();
  if (problem.mediumStack.size() > 0 && problem.mediumStack.top() != DefaultMedium) {
    IntID mediumID = problem.mediumStack.top();
    Medium *medium = mediumManager.getMedium(mediumID);

    if (medium == 0) {
      return false;
    }

    Pointf ro(ray.org[0], ray.org[1], ray.org[2]);

    float opticalPath = 0;
    double scatDist =
      medium->getPropagationDistance(ray.randEngine, ro, -node.exitantDirection, ray.elapsedTime, ray.tfar, opticalPath);

    problem.opticalDist += opticalPath;

    // look for artificial end points
    if (scatDist > ray.tfar && ray.geomID == unsigned(INVALID_ID)) {
      // this is an artificial stop -- make sure we don't keep going
      node.stopPath = true;
      node.pathDist = ray.propagationDistance + ray.tfar;
      node.hitDist = apis::RayMax;

      SpectralVector ior;
      medium->getRefractiveIndex(ro + (-0.5 * ray.tfar) * node.exitantDirection, ray.elapsedTime, false, ior);

      node.elapsedTime = ray.elapsedTime - ior[refWavelength] * (node.pathDist - ray.propagationDistance) / speedOfLight;

      // construct the node point exactly as far as we went
      // (no surface offsets)
      node.pathPoint =
        Pointf(ray.org[0] + ray.tfar * ray.dir[0], ray.org[1] + ray.tfar * ray.dir[1], ray.org[2] + ray.tfar * ray.dir[2]);
    } else {
      SpectralVector ior(nWavelengths);
      if (node.hitDist < (ray.propagationDistance + scatDist)) {
        node.pathDist = node.hitDist;
        node.pathPoint =
          Pointf(ray.org[0] + ray.tfar * ray.dir[0], ray.org[1] + ray.tfar * ray.dir[1], ray.org[2] + ray.tfar * ray.dir[2]);

        medium->getRefractiveIndex(ro + (-0.5 * node.hitDist) * node.exitantDirection, ray.elapsedTime, false, ior);

        node.elapsedTime = ray.elapsedTime - ior[refWavelength] * (node.pathDist - ray.propagationDistance) / speedOfLight;
      } else {
        node.pathDist = ray.propagationDistance + scatDist;
        node.pathPoint =
          Pointf(ray.org[0] + scatDist * ray.dir[0], ray.org[1] + scatDist * ray.dir[1], ray.org[2] + scatDist * ray.dir[2]);

        medium->getRefractiveIndex(ro + (-0.5 * scatDist) * node.exitantDirection, ray.elapsedTime, false, ior);

        node.elapsedTime = ray.elapsedTime - ior[refWavelength] * (node.pathDist - ray.propagationDistance) / speedOfLight;
      }
    }
  } else {
    // check for not hitting anything but having a non-infinite hit distance
    // this can happen when hitting the boundary of media
    if (ray.tfar < apis::RayMax && ray.geomID == unsigned(INVALID_ID)) {
      // this is an artificial stop -- make sure we don't keep going
      node.stopPath = true;
      node.pathDist = ray.propagationDistance + ray.tfar;
      node.hitDist = apis::RayMax;
      if (node.mediumStack.empty() || node.mediumStack.top() != ray.curMedium) {
        // we might have snuck in -- correct the stack
        node.mediumStack.push(ray.curMedium);
      }
    } else {
      // if we didn't scatter or hit a boundary,
      // then we just reached the hit point
      node.pathDist = node.hitDist;
    }

    if (!node.stopPath) {
      if (node.sceneID == EarthSceneID) {
        node.pathPoint = Pointf(
          ray.ecefOrigin[0] + ray.tfar * ray.ecefDirection[0], ray.ecefOrigin[1] + ray.tfar * ray.ecefDirection[1],
          ray.ecefOrigin[2] + ray.tfar * ray.ecefDirection[2]);
      } else {
        node.pathPoint =
          Pointf(ray.org[0] + ray.tfar * ray.dir[0], ray.org[1] + ray.tfar * ray.dir[1], ray.org[2] + ray.tfar * ray.dir[2]);
      }

      // track the elapsed time to arrive at the node after localization
      node.elapsedTime = ray.elapsedTime - ray.currentIOR * (node.pathDist - ray.propagationDistance) / speedOfLight;
    }
  }

  // return if we stopped the path
  if (node.stopPath) {
    node.rayOrigin =
      Pointf(ray.org[0] + ray.tfar * ray.dir[0], ray.org[1] + ray.tfar * ray.dir[1], ray.org[2] + ray.tfar * ray.dir[2]);
    node.rayDirection = Vectorf(ray.dir[0], ray.dir[1], ray.dir[2]);
    return false;
  }

  // grab the scratch space for the thread
  static thread_local Scratch scratch;
  scratch.update(nWavelengths);

  // check for scattering
  if (node.pathDist < node.hitDist) {
    node.geomID = INVALID_ID;

    // new ray is the same as the path point (no offset)
    node.rayOrigin = node.pathPoint;

    Eigen::Quaternionf &scatTransform = scratch.quaternion;

    static const Vectorf localForward(0, 0, 1);
    static thread_local Vectorf globalForward;
    globalForward = -1.f * node.exitantDirection;
    scatTransform.setFromTwoVectors(localForward, globalForward);

    Medium *medium = mediumManager.getMedium(node.mediumStack.top());
    if (medium == 0) return false;

    node.sampleWeight.resize(nWavelengths);
    node.sampleWeight.setData(1.0);

    float sampledMu = std::max(-1.f, std::min(1.f, medium->sampleScatteringPDF(node.rayOrigin, node.elapsedTime)));
    float th = acosf(sampledMu);
    float ph = 2.f * PI * canonical(ray.randEngine);
    if (sampledMu < 0.99999999) problem.scattered = true;

    Vectorf &localDir = scratch.vc[0];

    // compute the local direction
    localDir = {sinf(th) * cosf(ph), sinf(th) * sinf(ph), sampledMu};

    // rotate to the forward direction
    node.rayDirection = scatTransform * localDir;

    // we added a node
    ray.skip = false;

    return true;
  } else if (ray.geomID == unsigned(INVALID_ID)) {
    // didn't hit anything
    node.geomID = INVALID_ID;
    return false;
  }

  auto &sceneManager = MutableSingleton<SceneManager>();

  ray.collectMetaData = true;
  ray.hitMetaData.clear();

  // map the ray through the scene to generate a node
  sceneManager.setSceneMatID(ray);
  sceneManager.buildNodeFromRay(ray, node);

  bool mediumTransition = false;
  if (node.opticalProp) mediumTransition = node.opticalProp->isMediumTransition();

  // make sure we don't attempt to handle null or other non-standard surfaces
  if (!node.opticalProp || mediumTransition) {
    // pass through the surface
    node.rayOrigin = Pointf(
      ray.org[0] + (double(ray.tfar) + SURFACE_EPS) * ray.dir[0], ray.org[1] + (double(ray.tfar) + SURFACE_EPS) * ray.dir[1],
      ray.org[2] + (double(ray.tfar) + SURFACE_EPS) * ray.dir[2]);
    node.rayDirection = Vectorf(ray.dir[0], ray.dir[1], ray.dir[2]);

    // we just added a dummy node regardless
    if (!node.hasNullMixture)
      node.addExoSources = false;
    else
      node.addExoSources = true;

    if (mediumTransition) NodeGenerator::handleMediumTransition(problem, node, nWavelengths);

    return true;
  }

  if (problem.getNodeCount() > problem.getMaxPathLength()) {
    // this is an artificial stop -- make sure we don't keep going
    node.stopPath = true;
    return false;
  }

  // make sure the hit distance makes sense
  if (node.pathDist <= 0) {
    node.geomID = INVALID_ID;
    node.pathDist = std::numeric_limits<double>::max();
    node.hitDist = std::numeric_limits<double>::max();
    return false;
  }

  // sample the surface, store the probability
  bool reflected = false;
  node.sampleSurface(ray.randEngine, problem, reflected);

  // grab the original medium we traveled through
  IntID curMedium = DefaultMedium;
  if (!problem.mediumStack.empty()) curMedium = problem.mediumStack.top();

  // make sure we have the atmosphere to add the sun
  bool addExoSources = (curMedium == DefaultMedium);

  if (!addExoSources) {
    IntID backsideMedium;

    // see if there is a medium on the other side of this surface
    if (!node.hitBackside && node.opticalProp->hasBacksideMedium(backsideMedium))
      addExoSources = (backsideMedium == DefaultMedium);
  }

  // definitely trace the sun if we start in the atmosphere
  if (!problem.mediumStack.empty())
    if (problem.mediumStack.top() != DefaultMedium) {
      addExoSources = false;
    }

  if (!addExoSources && !node.mediumStack.empty()) {
    if (problem.mediumStack.top() != DefaultMedium) {
      // we also add for media without an interface
      if (!MutableSingleton<MediumManager>().getMedium(problem.mediumStack.top())->hasInterface()) {
        addExoSources = true;
      }
    }
  }

  // move the ray slightly away from the surface
  Vectorf offset = node.Ng;
  if (offset.dot(node.exitantDirection) < 0) offset *= -1.0f;
  if (reflected) {
    node.rayOrigin = node.pathPoint + SURFACE_EPS * offset;
  } else {
    node.rayOrigin = node.pathPoint - SURFACE_EPS * offset;
    NodeGenerator::handleMediumTransition(problem, node, nWavelengths);
  }

  if (!node.sampleWeight.array().allFinite())
    // absorbed
    return false;

  // flag the node for tracing exo sources
  node.addExoSources = addExoSources;

  return true;
}

} // namespace d5::core
