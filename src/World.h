#pragma once

#include <Microcosm/Geometry/FileMTL>
#include <Microcosm/Geometry/FileOBJ>
#include <Microcosm/Geometry/Mesh>
#include <Microcosm/Render/TriangleMesh>
#include <Microcosm/stbi>

#include "dirsig5b/World.h"

class World final : public d5b::World {
public:
  void initialize();

  [[nodiscard]] bool intersect(d5b::Random &random, d5b::Ray ray, d5b::LocalSurface &localSurface) const override;

  // TODO Visibility instead of boolean.
  [[nodiscard]] bool intersect(d5b::Random &random, d5b::Ray ray) const override;

  void directLightsForVertex(
    d5b::Random &random,
    const d5b::Vertex &vertex,
    const d5b::SpectralVector &wavelength,
    std::vector<d5b::DirectLight> &directLights) const override;

  mi::geometry::FileOBJ fileOBJ;
  mi::geometry::FileMTL fileMTL;
  mi::render::TriangleMesh mesh;

  struct Textures {
    std::optional<mi::stbi::ImageU8> albedo;
    std::optional<mi::stbi::ImageU8> normal;
  };
  std::map<int16_t, Textures> texturesForMaterial;
};
