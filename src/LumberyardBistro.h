#pragma once

#include <Microcosm/Geometry/FileMTL>
#include <Microcosm/Geometry/FileOBJ>
#include <Microcosm/Geometry/Mesh>
#include <Microcosm/Render/TriangleMesh>
#include <Microcosm/stbi>

#include "dirsig5b/World.h"

class LumberyardBistro final : public d5b::World {
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

  mi::geometry::FileOBJ fileOBJ;
  mi::geometry::FileMTL fileMTL;
  mi::render::TriangleMesh mesh;

  struct Textures {
    std::optional<mi::stbi::ImageU8> albedo;
    std::optional<mi::stbi::ImageU8> normal;
    std::optional<mi::stbi::ImageU8> opacity;
    bool isLeaf{false};
    bool isMetal{false};
    bool isCloth{false};
  };
  std::map<int16_t, Textures> texturesForMaterial;
};
