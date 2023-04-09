#include "dirsig5b/World.h"

namespace d5b {

bool World::isVisible(Random &random, Ray ray, const SpectralVector &, SpectralVector &transmission) const {
  // TODO Handle participating media.
  LocalSurface localSurface;
  if (ray.maxParam > ray.minParam && intersect(random, ray, localSurface)) {
    transmission = 0;
    return false;
  } else {
    transmission = 1;
    return true;
  }
}
void World::directLightsForVertex(
  Random &, const Vertex &, const SpectralVector &, std::vector<DirectLight> &directLights) const {
  // By default, no direct lights.
  directLights.clear();
}

void World::infiniteLightContributionForEscapedRay(Random &, Ray, const SpectralVector &, SpectralVector &emission) const {
  // By default, no infinite lights.
  emission = 0;
}

const Medium *World::mediumLookup(Vector3) const {
  return nullptr; // By default, no participating media.
}

} // namespace d5b
