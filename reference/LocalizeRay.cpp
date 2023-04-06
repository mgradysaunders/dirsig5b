#include "dirsig5/internal/core/paths/LocalizeRay.h"

#include <fstream>

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/UTMUPS.hpp>

#include "dirsig5/common/Constants.h"
#include "dirsig5/common/Functional.h"
#include "dirsig5/common/IntID.h"
#include "dirsig5/internal/core/components/Bvh.h"
#include "dirsig5/internal/core/components/PathInfo.h"

namespace d5::core {

AtmLayers *LocalizeRay::layers = nullptr;
bool LocalizeRay::initialized = false;

void LocalizeRay::run(apis::Ray &ray, bool propagateToEllipsoid) try {
  static const double scenePad = 0.1; // give ourselves a little space

  // grab the scene manager
  auto &sm = MutableSingleton<SceneManager>();

  // if we have double-precision scene coordinates, convert to ecef
  // for propagation (if we have atm layers) or just get to the scene
  if (ray.coordinates == apis::DoubleScene) {
    // if layers, we need to propagate through an atmosphere, convert
    if (LocalizeRay::layers != 0 || propagateToEllipsoid) {
      // convert to ECEF
      sm.convertENUtoECEF(ray.sceneID, ray.gOrigin, ray.ecefOrigin);
      sm.convertENUtoECEF(ray.sceneID, ray.gDirection, ray.ecefDirection, true);

      // change coordinate system
      ray.coordinates = apis::DoubleEarth;
    }
    // if we're not going to propagate we can just project to the
    // top of all the scenes
    else {
      // find the top of the scene box
      double sceneHeight = sm.getMaxHeight(ray.sceneID) + scenePad;

      // check if we're headed down
      double diff = 0, t = 0;
      if (sceneHeight < ray.gOrigin[2] && ray.gDirection[2] < 0) {
        // we just need to propagate to the top of the box
        diff = sceneHeight - ray.gOrigin[2];
        t = diff / ray.gDirection[2];
      }

      // first see if we're going to make it there,
      // check for interstitial objects
      if (sm.hasInterstitials()) {
        // limit the distance if we'd hit the scene top
        float maxT = std::min(float(t), ray.tfar);

        if (checkInterstitials(ray, maxT)) {
          // we hit one of the interstitial objects,
          return;
        }
      }

      // grab the nominal refractive index
      ray.currentIOR = sm.getRefractiveIndex(ray.sceneID);

      if (sceneHeight < ray.gOrigin[2] && ray.gDirection[2] < 0) {
        ray.org[0] = static_cast<float>(ray.gOrigin[0] + t * ray.gDirection[0]);
        ray.org[1] = static_cast<float>(ray.gOrigin[1] + t * ray.gDirection[1]);
        ray.org[2] = static_cast<float>(ray.gOrigin[2] + t * ray.gDirection[2]);

        ray.propagationDistance += t;
        ray.elapsedTime -= defaultAtmPathTime(ray.gOrigin, ray.gOrigin + t * ray.gDirection, ray.gDirection[2]);
      } else {
        // just downgrade
        ray.org[0] = static_cast<float>(ray.gOrigin[0]);
        ray.org[1] = static_cast<float>(ray.gOrigin[1]);
        ray.org[2] = static_cast<float>(ray.gOrigin[2]);
      }

      ray.dir[0] = static_cast<float>(ray.gDirection[0]);
      ray.dir[1] = static_cast<float>(ray.gDirection[1]);
      ray.dir[2] = static_cast<float>(ray.gDirection[2]);

      // we're now in single precision
      ray.coordinates = apis::SingleScene;

      // check if we're nowhere near the scene itself
      Pointf bbmin, bbmax;
      sm.getBounds(ray.sceneID, bbmin, bbmax);
      bbmin -= Pointf(scenePad, scenePad, scenePad);
      bbmax += Pointf(scenePad, scenePad, scenePad);

      Point oScene = (0.5 * (bbmin + bbmax)).cast<double>();
      Point oRay(ray.org[0], ray.org[1], ray.org[2]);
      double r2Scene = (bbmax - bbmin).squaredNorm() / 4.;
      double r2Ray = (oRay - oScene).squaredNorm();
      if (r2Ray > r2Scene) {
        // we appear to be outside the bounding sphere still, get closer?
        double rScene = sqrt(r2Scene);

        // attempt to intersect
        Vector dRay(ray.dir[0], ray.dir[1], ray.dir[2]);
        double tSphere = intersectRaySphere((oRay - oScene), dRay, rScene);

        if (tSphere > 0) {
          // constrain to the top/bottom of the bounding box
          if (ray.org[2] + tSphere * ray.dir[2] > bbmax[2]) tSphere = (bbmax[2] - ray.org[2]) / ray.dir[2];
          if (ray.org[2] + tSphere * ray.dir[2] > bbmax[2]) tSphere = (bbmax[2] - ray.org[2]) / ray.dir[2];

          // propagate to the sphere (to do fix precision management)
          ray.org[0] = static_cast<float>(oRay[0] + tSphere * dRay[0]);
          ray.org[1] = static_cast<float>(oRay[1] + tSphere * dRay[1]);
          ray.org[2] = static_cast<float>(oRay[2] + tSphere * dRay[2]);

          ray.propagationDistance += tSphere;
          ray.elapsedTime -= tSphere / (speedOfLight / ray.currentIOR);
        }
      }
    }
  }

  // If we're in ECEF, convert to scene local coordinates (single-precision)
  if (ray.coordinates == apis::DoubleEarth) {
    // grab the scenemanager and projection
    auto &sm = MutableSingleton<SceneManager>();

    // first see if we're going to make it there,
    // check for interstitial objects
    if (sm.hasInterstitials())
      if (checkInterstitials(ray, ray.tfar)) return;

    // get and pad the top of the scene(s)
    double altTop;
    if (propagateToEllipsoid)
      altTop = 0.0;
    else
      altTop = sm.getMaxAltitude() + scenePad;

    // if we have a varying atmosphere, propagate through it
    if (LocalizeRay::layers != 0) {
      double propagationTime = 0.;
      double refractiveIndex = 1.;
      ray.propagationDistance +=
        LocalizeRay::layers->propagate(ray.ecefOrigin, ray.ecefDirection, altTop, propagationTime, refractiveIndex);

      ray.currentIOR = refractiveIndex;
      ray.elapsedTime -= propagationTime;

      // convert to ENU
      sm.convertECEFtoENU(ray.sceneID, ray.ecefOrigin, ray.gOrigin);
      sm.convertECEFtoENU(ray.sceneID, ray.ecefDirection, ray.gDirection, true);
    } else {
      // only need to propagate down to the bounding sphere
      const double boundingSphereR = WGS84EquatorialRadius + altTop;

      Point sclOrigin = ray.ecefOrigin;
      sclOrigin[2] /= EARTH_ECCENTRICITY;

      Vector sclDirection = ray.ecefDirection;
      sclDirection[2] /= EARTH_ECCENTRICITY;
      sclDirection.normalize();

      // see if we're outside the sphere
      if (sclOrigin.squaredNorm() > boundingSphereR * boundingSphereR) {
        // trace ray to the bounding sphere if we need to
        double t = intersectRaySphere(sclOrigin, sclDirection, boundingSphereR);

        // get down to the scene boundary if its far enough to care
        if (t > 1000 || propagateToEllipsoid) {
          sclOrigin += t * sclDirection;

          ray.ecefOrigin = sclOrigin;
          ray.ecefOrigin[2] *= EARTH_ECCENTRICITY;

          ray.ecefDirection = sclDirection;
          ray.ecefDirection[2] *= EARTH_ECCENTRICITY;
          ray.ecefDirection.normalize();

          ray.propagationDistance += t;
          ray.elapsedTime -= t / speedOfLight;

          // convert to ENU
          sm.convertECEFtoENU(ray.sceneID, ray.ecefOrigin, ray.gOrigin);
          sm.convertECEFtoENU(ray.sceneID, ray.ecefDirection, ray.gDirection, true);

          // update the propagation time based on the default atm
          ray.elapsedTime -= defaultAtmPathTime(ray.gOrigin, ray.gOrigin - t * ray.gDirection, ray.gDirection[2]);

        } else {
          // convert to ENU
          sm.convertECEFtoENU(ray.sceneID, ray.ecefOrigin, ray.gOrigin);
          sm.convertECEFtoENU(ray.sceneID, ray.ecefDirection, ray.gDirection, true);
        }
      } else {
        // convert to ENU
        sm.convertECEFtoENU(ray.sceneID, ray.ecefOrigin, ray.gOrigin);
        sm.convertECEFtoENU(ray.sceneID, ray.ecefDirection, ray.gDirection, true);
      }

      // don't assume a refractive index without it being defined
      ray.currentIOR = 1.0;
    }

    // downcast
    ray.org[0] = static_cast<float>(ray.gOrigin[0]);
    ray.org[1] = static_cast<float>(ray.gOrigin[1]);
    ray.org[2] = static_cast<float>(ray.gOrigin[2]);

    ray.dir[0] = static_cast<float>(ray.gDirection[0]);
    ray.dir[1] = static_cast<float>(ray.gDirection[1]);
    ray.dir[2] = static_cast<float>(ray.gDirection[2]);

    // we're now in single precision
    ray.coordinates = apis::SingleScene;
  }

} catch (std::exception &e) {
  static std::mutex mutex;
  mutex.lock();
  std::cerr << e.what() << std::endl;
  abort();
}

void LocalizeRay::setGlobal(apis::Ray &ray) {
  // grab the scenemanager and projection
  auto &sm = MutableSingleton<SceneManager>();

  if (ray.coordinates == apis::DoubleEarth) {
    // make sure we have global
    sm.convertECEFtoENU(ray.sceneID, ray.ecefOrigin, ray.gOrigin);
    sm.convertECEFtoENU(ray.sceneID, ray.ecefDirection, ray.gDirection, true);
    return;
  }

  if (ray.coordinates == apis::DoubleScene) {
    // define single precision too
    ray.org[0] = ray.gOrigin[0];
    ray.org[1] = ray.gOrigin[1];
    ray.org[2] = ray.gOrigin[2];
    ray.dir[0] = ray.gDirection[0];
    ray.dir[1] = ray.gDirection[1];
    ray.dir[2] = ray.gDirection[2];
  } else {
    assert(ray.coordinates == apis::SingleScene);
    ray.gOrigin[0] = ray.org[0];
    ray.gOrigin[1] = ray.org[1];
    ray.gOrigin[2] = ray.org[2];
    ray.gDirection[0] = ray.dir[0];
    ray.gDirection[1] = ray.dir[1];
    ray.gDirection[2] = ray.dir[2];
  }

  Point dxyz;
  if (ray.proxyParent >= 0) {
    sm.convertENUtoECEF(ray.proxyParent, ray.gOrigin, ray.ecefOrigin);
    sm.convertENUtoECEF(ray.proxyParent, ray.gDirection, ray.ecefDirection, true);
  } else {
    sm.convertENUtoECEF(ray.sceneID, ray.gOrigin, ray.ecefOrigin);
    sm.convertENUtoECEF(ray.sceneID, ray.gDirection, ray.ecefDirection, true);
  }

  ray.coordinates = apis::DoubleEarth;
}

void LocalizeRay::initialize() {
  if (LocalizeRay::initialized) return;

  // just use the MLS profiles for now
  LocalizeRay::layers = new AtmLayers;

  LocalizeRay::initialized = true;

  // sign up for spectral state updates
  auto &stateManager = MutableSingleton<StateManager>();
  stateManager.registerSpectralStateUpdates([](auto &&...args) { LocalizeRay::updateState(D5_FWD(args)...); });
}

void LocalizeRay::updateState(const State *_state) {
  // grab some state data
  size_t refWLI = _state->spectralStates->at(_state->spectralStateIndex).referenceWavelengthIndex;
  const std::vector<double> &wavelengths = _state->spectralStates->at(_state->spectralStateIndex).wavelengthSamples;

  // grab the current reference wavelength [um]
  float refWL = wavelengths[refWLI];

  // set the profiles
  auto &atmManager = MutableSingleton<AtmosphereManager>();

  const apis::plugins::AtmosphereProfiles::Out &profiles = atmManager.getProfiles();

  if (profiles.temperatureCProfile.size() > 0) {
    if (!LocalizeRay::initialized) {
      LocalizeRay::initialize();
    }

    AtmLayers::DataProfile dp(
      profiles.altitudeM, profiles.temperatureCProfile, profiles.pressureMBProfile, profiles.dewPointCProfile);

    LocalizeRay::layers->setProfile(dp);

  } else if (profiles.builtInProfiles != apis::plugins::NONE) {
    if (!LocalizeRay::initialized) {
      LocalizeRay::initialize();
    }

    AtmLayers::BuiltInProfile bip(profiles.builtInProfiles);
    LocalizeRay::layers->setProfile(bip);
  }

  if (LocalizeRay::initialized) {
    // initialize the layers themselves
    LocalizeRay::layers->initLayers(refWL, AtmLayers::CIDDOR);
  }
}

} // namespace d5::core
