#pragma once

#include <Microcosm/Render/TriangleMesh>
#include <Microcosm/stbi>

#include "dirsig5b/Simulation.h"

class FractalWorld final : public d5b::World {
public:
  void initialize();

  [[nodiscard]] bool intersect(d5b::Random &random, d5b::Ray ray, d5b::LocalSurface &localSurface) const override;

  void directLightsForVertex(
    d5b::Random &random,
    const d5b::Vertex &vertex,
    const d5b::SpectralVector &wavelength,
    std::vector<d5b::DirectLight> &directLights) const override;

  void infiniteLightContributionForEscapedRay(
    d5b::Random &random, d5b::Ray ray, const d5b::SpectralVector &wavelength, d5b::SpectralVector &emission) const override;
};
