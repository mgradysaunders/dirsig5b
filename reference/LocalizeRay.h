#pragma once

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/UTMUPS.hpp>

#include "dirsig5/apis/Ray.h"
#include "dirsig5/common/State.h"
#include "dirsig5/internal/core/components/AtmLayers.h"
#include "dirsig5/internal/core/managers/SingletonManager.h"

namespace d5::core {

/**
 * This class provides a function to localize a ray, i.e. propagate it
 * from a global origin/direction to a scene local equivalent.
 */
class LocalizeRay {
public:
  static void run(apis::Ray &ray, bool propagateToEllipsoid = false);

  /**
   * Set global ECEF coordinates for the current local scene. Returns
   * the altitude.
   */
  static void setGlobal(apis::Ray &ray);

  static void updateState(const State *_state);

private:
  static void initialize();

  static bool initialized;

  static AtmLayers *layers;
};

/**
 * Shared lambda for checking for intersections with interstitials
 */
static auto checkInterstitials = [](apis::Ray &ray, float maxT) {
  static thread_local apis::EmbreeRay er;
  static thread_local apis::Ray intray(&er);
  intray.copy(ray);

  // list of interstitial bounding boxes encountered in tnear order
  BVH::ObjList interstitialList;

  // make sure we're localized in global coords
  LocalizeRay::setGlobal(intray);

  // grab the scene manager
  auto &sm = MutableSingleton<SceneManager>();
  bool hit = false;
  if (sm.intersectInterstitials(intray, &interstitialList)) {
    double lastDist = 0.0;
    auto iter = interstitialList.begin();
    for (; iter != interstitialList.end(); ++iter) {
      if (iter->tfar < 0) continue;
      if (iter->tnear > maxT) continue;

      // we can be in a box, so make sure we don't go backward
      double curDist = std::max(0.0, iter->tnear - lastDist);
      lastDist += curDist;

      // propagate to the bounds and try to intersect
      intray.ecefOrigin += curDist * intray.ecefDirection;
      intray.propagationDistance += curDist;
      intray.elapsedTime -= curDist / (speedOfLight / intray.currentIOR);

      intray.sceneID = iter->obj->getParentSceneID();

      // convert to ENU
      sm.convertECEFtoENU(intray.sceneID, intray.ecefOrigin, intray.gOrigin);
      sm.convertECEFtoENU(intray.sceneID, intray.ecefDirection, intray.gDirection, true);

      // update single precision
      intray.org = intray.gOrigin.cast<float>();
      intray.dir = intray.gDirection.cast<float>();

      // we're now in single precision
      intray.coordinates = apis::SingleScene;

      intray.geomID = INVALID_ID;

      intray.tnear = 0;
      intray.tfar = iter->tfar - curDist;

      sm.sceneIntersect(intray);

      if ( // check for hit and make sure the distance is in bounds
        intray.geomID != unsigned(INVALID_ID) && intray.tfar < (maxT - iter->tnear - 1e-6)) {

        ray.copy(intray);

        // set a new maximum, but continue looking for closer hits...
        maxT = intray.tfar + lastDist;

        hit = true;
      }

      // back to ecef
      intray.coordinates = apis::DoubleEarth;
    }
    return hit;
  }

  return false;
};

} // namespace d5::core
