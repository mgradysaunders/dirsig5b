#pragma once

#include <Microcosm/Render/TriangleMesh>
#include <Microcosm/stbi>
#include <list>

#include "dirsig5b/Simulation.h"

class ForestWorld final : public d5b::World {
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

  struct Tree {
    mi::stbi::ImageU8 leafOpacity;
    mi::stbi::ImageU8 leafAlbedo;
    mi::stbi::ImageU8 barkAlbedo;
    std::list<mi::render::TriangleMesh> meshes;
  };

  struct TreeInstance {
    const Tree *tree{};
    const mi::render::TriangleMesh *mesh{};
    d5b::Transform transform{};
  };

  std::list<Tree> trees;
  std::vector<TreeInstance> treeInstances;
};
