#pragma once

#include "dirsig5b/Lights.h"
#include "dirsig5b/LocalSurface.h"
#include "dirsig5b/Medium.h"
#include "dirsig5b/Ray.h"
#include "dirsig5b/Vertex.h"

namespace d5b {

class D5B_API World {
public:
  virtual ~World() = default;

  [[nodiscard]] virtual bool intersect(Random &random, Ray ray, LocalSurface &localSurface) const = 0;

  [[nodiscard]] virtual bool
  isVisible(Random &random, Ray ray, const SpectralVector &wavelength, SpectralVector &transmission) const;

  virtual void directLightsForVertex(
    Random &random, const Vertex &vertex, const SpectralVector &wavelength, std::vector<DirectLight> &directLights) const;

  virtual void infiniteLightContributionForEscapedRay(
    Random &random, Ray ray, const SpectralVector &wavelength, SpectralVector &emission) const;

  [[nodiscard]] virtual const Medium *mediumLookup(Vector3 position) const;
};

} // namespace d5b
