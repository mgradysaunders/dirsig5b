#include "World.h"

#include <filesystem>
#include <iostream>

void World::initialize() {
  fileOBJ = mi::geometry::FileOBJ("/home/michael/Documents/Scenes/lumberyard-bistro/exterior.obj");
  fileMTL = mi::geometry::FileMTL("/home/michael/Documents/Scenes/lumberyard-bistro/exterior.mtl");
  mesh.buildFrom(mi::geometry::Mesh(fileOBJ), 2);
  for (int16_t materialIndex : mesh.materials) {
    if (texturesForMaterial.find(materialIndex) == texturesForMaterial.end()) {
      auto &textures = texturesForMaterial[materialIndex];
      auto &material = fileMTL.materials.at(fileOBJ.metadata.materialNames.at(materialIndex));
      if (material.ambientTexture) {
        std::filesystem::path filePath =
          std::filesystem::path("/home/michael/Documents/Scenes/lumberyard-bistro") / *material.ambientTexture;
        std::cout << "Loading " << filePath.string() << "..." << std::endl;
        textures.albedo = mi::stbi::loadU8(filePath.string());
      }
      if (material.bumpTexture) {
        std::filesystem::path filePath =
          std::filesystem::path("/home/michael/Documents/Scenes/lumberyard-bistro") / *material.bumpTexture;
        std::cout << "Loading " << filePath.string() << "..." << std::endl;
        textures.normal = mi::stbi::loadU8(filePath.string(), 3);
      }
    }
  }
}

bool World::intersect(d5b::Random &, d5b::Ray ray, d5b::LocalSurface &localSurface) const {
  mi::render::TriangleMesh::Location location;
  if (mesh.rayTest(mi::Ray3f(ray.org, ray.dir, ray.minParam, ray.maxParam), location)) {
    auto materialIndex = mesh.materials.at(location.index);
    // auto &material = fileMTL.materials.at(fileOBJ.metadata.materialNames.at(materialIndex));
    localSurface.position = location.point;
    localSurface.texcoord = location.shading.texcoord;
    localSurface.tangents[0] = location.shading.tangents.col(0);
    localSurface.tangents[1] = location.shading.tangents.col(1);
    localSurface.normal = location.shading.normal;
    auto itr = texturesForMaterial.find(materialIndex);
    if (itr != texturesForMaterial.end() && itr->second.normal) {
      auto texcoord = location.shading.texcoord;
      auto &normal = *itr->second.normal;
      int y = normal.size(0) * (1 - mi::fract(texcoord[1]));
      int x = normal.size(1) * mi::fract(texcoord[0]);
      mi::Vector3d localNormal = {
        normal(y, x, 0) * (1.0 / 255.0) * 2 - 1, //
        normal(y, x, 1) * (1.0 / 255.0) * 2 - 1, //
        normal(y, x, 2) * (1.0 / 255.0) * 2 - 1};
      localNormal = mi::fastNormalize(localNormal);
      localSurface.normal = location.shading.normal * localNormal[2] +      //
                            location.shading.tangents.col(0) * localNormal[0] + //
                            location.shading.tangents.col(1) * localNormal[1];
    }
    localSurface.scatteringProvider =
      [&, itr, materialIndex, texcoord = location.shading.texcoord](const d5b::SpectralVector &, d5b::Scattering &scattering) {
        scattering.setLambertDiffuse(0.333, 0.0);
        if (itr != texturesForMaterial.end() && itr->second.albedo) {
          auto &albedo = *itr->second.albedo;
          int y = albedo.size(0) * (1 - mi::fract(texcoord[1]));
          int x = albedo.size(1) * mi::fract(texcoord[0]);
          mi::Vector3d color = {
            mi::decodeSRGB(albedo(y, x, 0) * (1.0 / 255.0)), //
            mi::decodeSRGB(albedo(y, x, 1) * (1.0 / 255.0)), //
            mi::decodeSRGB(albedo(y, x, 2) * (1.0 / 255.0))};
          scattering.multiply({color[0], color[1], color[2], 1.0});
        }
      };
    return true;
  }
  return false;
}

// TODO Visibility instead of boolean
bool World::intersect(d5b::Random &, d5b::Ray ray) const {
  return mesh.rayTestShadowOnly(mi::Ray3f(ray.org, ray.dir, ray.minParam, ray.maxParam)).has_value();
}

void World::directLightsForVertex(
  d5b::Random &random,
  const d5b::Vertex &vertex,
  const d5b::SpectralVector &wavelength,
  std::vector<d5b::DirectLight> &directLights) const {
  auto &sun = directLights.emplace_back();
  d5b::SpectralVector emissionCurve{wavelength.shape};
  for (size_t i = 0; i < wavelength.size(); i++) {
    emissionCurve[i] = 16 * mi::normalizedBlackbodyRadiance(5500.0, wavelength[i]);
  }
  sun.sampleSolidAngleEmission = [emissionCurve = std::move(emissionCurve)](
                                   d5b::Random &random, d5b::Vector3, d5b::Vector3 &directionToLight, double &distanceToLight,
                                   d5b::SpectralVector &emission) -> double {
    directionToLight = d5b::normalize(d5b::Vector3(-1, 3, 1));
    distanceToLight = d5b::Inf;
    emission.assign(emissionCurve);
    return 1;
  };
}
