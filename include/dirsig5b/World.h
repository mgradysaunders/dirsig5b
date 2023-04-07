#pragma once

#include "dirsig5b/LocalSurface.h"
#include "dirsig5b/Medium.h"
#include "dirsig5b/Ray.h"
#include "dirsig5b/Vertex.h"

namespace d5b {

class D5B_API DirectLight final {
public:
  std::function<double(
    Random &random, Vector3 position, Vector3 &directionToLight, double &distanceToLight, SpectralVector &emission)>
    sampleSolidAngleEmission;

  size_t numSubSamples{1};
};

class D5B_API World {
public:
  virtual ~World() = default;

  [[nodiscard]] virtual const Medium *mediumLookup(Vector3) const { return nullptr; }

  [[nodiscard]] virtual bool intersect(Random &random, Ray ray, LocalSurface &localSurface) const = 0;

  // TODO Visibility instead of boolean
  [[nodiscard]] virtual bool intersect(Random &random, Ray ray) const {
    LocalSurface localSurface;
    return intersect(random, ray, localSurface);
  }

  virtual void directLightsForVertex(
    Random &random, const Vertex &vertex, const SpectralVector &wavelength, std::vector<DirectLight> &directLights) const = 0;
};

} // namespace d5b
