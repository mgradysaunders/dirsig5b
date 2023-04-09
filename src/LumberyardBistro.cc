#include "LumberyardBistro.h"

#include <Microcosm/Render/Illuminant>
#include <Microcosm/Render/Microsurface>
#include <filesystem>
#include <iostream>

void LumberyardBistro::initialize() {
  fileOBJ = mi::geometry::FileOBJ("/home/michael/Documents/Scenes/lumberyard-bistro/exterior.obj");
  fileMTL = mi::geometry::FileMTL("/home/michael/Documents/Scenes/lumberyard-bistro/exterior.mtl");
  mesh.buildFrom(mi::geometry::Mesh(fileOBJ), 2);
  for (int16_t materialIndex : mesh.materials) {
    if (texturesForMaterial.find(materialIndex) == texturesForMaterial.end()) {
      auto &textures = texturesForMaterial[materialIndex];
      auto &material = fileMTL.materials.at(fileOBJ.metadata.materialNames.at(materialIndex));
      auto load = [](const std::string &filename) -> mi::stbi::ImageU8 {
        auto path{std::filesystem::path("/home/michael/Documents/Scenes/lumberyard-bistro") / filename};
        std::cout << "Loading " << path.string() << "..." << std::endl;
        return mi::stbi::loadU8(path.string());
      };
      if (material.ambientTexture) {
        auto &filename = *material.ambientTexture;
        textures.albedo = load(filename);
        textures.isMetal = filename.find("Metal") != std::string::npos || //
                           filename.find("metal") != std::string::npos;
        textures.isCloth = filename.find("Fabric") != std::string::npos || //
                           filename.find("fabric") != std::string::npos;
      } else if (material.diffuseTexture) {
        auto &filename = *material.diffuseTexture;
        textures.albedo = load(filename);
        textures.isMetal = filename.find("Metal") != std::string::npos || //
                           filename.find("metal") != std::string::npos;
        textures.isCloth = filename.find("Fabric") != std::string::npos || //
                           filename.find("fabric") != std::string::npos;
      }
      if (material.bumpTexture) {
        textures.normal = load(*material.bumpTexture);
      }
      if (material.opacityTexture) {
        textures.opacity = load(*material.opacityTexture);
        textures.isLeaf = material.opacityTexture->find("Natural") != std::string::npos;
      }
    }
  }
}

bool LumberyardBistro::intersect(d5b::Random &random, d5b::Ray ray, d5b::LocalSurface &localSurface) const {
  mi::render::TriangleMesh::Location location;
  if (auto param = mesh.rayTest(mi::Ray3f(ray.org, ray.dir, ray.minParam, ray.maxParam), location)) {
    auto materialIndex = mesh.materials.at(location.index);
    auto &material = fileMTL.materials.at(fileOBJ.metadata.materialNames.at(materialIndex));
    localSurface.position = location.point;
    localSurface.texcoord = location.shading.texcoord;
    localSurface.tangents[0] = location.shading.tangents.col(0);
    localSurface.tangents[1] = location.shading.tangents.col(1);
    localSurface.normal = location.shading.normal;
    auto texcoord = location.shading.texcoord;
    auto itr = texturesForMaterial.find(materialIndex);
#if 1
    if (itr != texturesForMaterial.end() && itr->second.normal) {
      auto &normal = *itr->second.normal;
      int y = normal.size(0) * (1 - mi::fract(texcoord[1]));
      int x = normal.size(1) * mi::fract(texcoord[0]);
      mi::Vector3d localNormal = {
        normal(y, x, 0) * (1.0 / 255.0) * 2 - 1, //
        normal(y, x, 1) * (1.0 / 255.0) * 2 - 1, //
        normal(y, x, 2) * (1.0 / 255.0) * 2 - 1};
      localNormal = mi::fastNormalize(localNormal);
      localSurface.normal = mi::fastNormalize(
        location.shading.normal * localNormal[2] +                             //
        mi::fastNormalize(location.shading.tangents.col(0)) * localNormal[0] + //
        mi::fastNormalize(location.shading.tangents.col(1)) * localNormal[1]);
    }
#endif
    mi::Vector4d color{1, 1, 1, 1};
    if (itr != texturesForMaterial.end() && itr->second.albedo) {
      auto &albedo = *itr->second.albedo;
      int y = albedo.size(0) * (1 - mi::fract(texcoord[1]));
      int x = albedo.size(1) * mi::fract(texcoord[0]);
      color[0] = mi::decodeSRGB(albedo(y, x, 0) * (1.0 / 255.0));
      color[1] = mi::decodeSRGB(albedo(y, x, 1) * (1.0 / 255.0));
      color[2] = mi::decodeSRGB(albedo(y, x, 2) * (1.0 / 255.0));
      color[3] = 1;
    }
    if (itr != texturesForMaterial.end() && itr->second.opacity) {
      auto &opacity = *itr->second.opacity;
      int y = opacity.size(0) * (1 - mi::fract(texcoord[1]));
      int x = opacity.size(1) * mi::fract(texcoord[0]);
      if (opacity(y, x, 0) == 0) {
        ray.minParam = *param + 1e-3;
        return intersect(random, ray, localSurface);
      }
    }
    localSurface.scatteringProvider = [&, itr, color](const d5b::SpectralVector &wavelength, d5b::Scattering &scattering) {
      auto colorToSpectrum = [&](mi::Vector3d c) {
        d5b::SpectralVector spectrum{wavelength.shape};
        for (size_t i = 0; i < wavelength.size(); i++) {
          spectrum[i] = 0;
        }
        for (size_t i = 0; i < wavelength.size(); i++) {
          spectrum[i] += c[0] * std::exp(-mi::sqr((wavelength[i] - 0.65) / 0.06));
          spectrum[i] += c[1] * std::exp(-mi::sqr((wavelength[i] - 0.55) / 0.06));
          spectrum[i] += c[2] * std::exp(-mi::sqr((wavelength[i] - 0.45) / 0.06));
        }
        return spectrum;
      };
      if (itr != texturesForMaterial.end() && itr->second.isMetal) {
        mi::render::ConductiveMicrosurface microsurface;
        microsurface.roughness = {0.15, 0.15};
        scattering = {
          [=](mi::Vector3d omegaO, mi::Vector3d omegaI, d5b::SpectralVector &f) {
            if (mi::signbit(omegaO[2]) == mi::signbit(omegaI[2]))
              f = microsurface.singleScatter(omegaO, omegaI).value;
            else
              f = 0;
          },
          [=](mi::Vector3d omegaO, mi::Vector3d omegaI) -> double {
            if (mi::signbit(omegaO[2]) == mi::signbit(omegaI[2]))
              return microsurface.singleScatter(omegaO, omegaI).valuePDF;
            else
              return 0;
          },
          [=](d5b::Random &random, mi::Vector3d omegaO, mi::Vector3d &omegaI, d5b::SpectralVector &beta) -> double {
            omegaI = microsurface.singleScatterSample(random, omegaO);
            if (mi::signbit(omegaO[2]) == mi::signbit(omegaI[2])) {
              auto [f, p] = microsurface.singleScatter(omegaO, omegaI);
              beta *= f / p;
              return p;
            } else {
              beta = 0;
              return 0;
            }
          },
        };
        scattering.multiply(colorToSpectrum(color));
      } else {
        d5b::Scattering diffuse;
        if (itr != texturesForMaterial.end() && (itr->second.isLeaf || itr->second.isCloth)) {
          diffuse.setLambertDiffuse(0.333, 0.333);
        } else {
          diffuse.setDisneyDiffuse(0.3, 0.333, 0.2, 0.0);
        }
        diffuse.multiply(colorToSpectrum(color));
        mi::render::DielectricMicrosurface microsurface;
        microsurface.roughness = {0.2, 0.2};
        if (material.specular) {
          d5b::Scattering spec = {
            [=](mi::Vector3d omegaO, mi::Vector3d omegaI, d5b::SpectralVector &f) {
              if (mi::signbit(omegaO[2]) == mi::signbit(omegaI[2]))
                f = microsurface.singleScatter(omegaO, omegaI).value;
              else
                f = 0;
            },
            [=](mi::Vector3d omegaO, mi::Vector3d omegaI) -> double {
              if (mi::signbit(omegaO[2]) == mi::signbit(omegaI[2]))
                return microsurface.singleScatter(omegaO, omegaI).valuePDF;
              else
                return 0;
            },
            [=](d5b::Random &random, mi::Vector3d omegaO, mi::Vector3d &omegaI, d5b::SpectralVector &beta) -> double {
              omegaI = microsurface.singleScatterSample(random, omegaO);
              if (mi::signbit(omegaO[2]) == mi::signbit(omegaI[2])) {
                auto [f, p] = microsurface.singleScatter(omegaO, omegaI);
                beta *= f / p;
                return p;
              } else {
                beta = 0;
                return 0;
              }
            },
          };
          spec.multiply(colorToSpectrum(*material.specular));
          scattering.setLinearMixture({{1.0, diffuse}, {1.0, spec}});
        } else {
          scattering = std::move(diffuse);
        }
      }
    };
    return true;
  }
  return false;
}

void LumberyardBistro::directLightsForVertex(
  d5b::Random &random,
  const d5b::Vertex &vertex,
  const d5b::SpectralVector &wavelength,
  std::vector<d5b::DirectLight> &directLights) const {

  {
    d5b::SpectralVector emission{wavelength.shape};
    for (size_t i = 0; i < wavelength.size(); i++)
      emission[i] = mi::render::IlluminantD(mi::convertCCTToXY(5500.0f), wavelength[i]);
    auto &sun = directLights.emplace_back();
    sun.importanceSampleSolidAngle = [emission = std::move(emission)](
                                       d5b::Random &random, d5b::Vector3, d5b::Vector3 &direction, double &distance,
                                       d5b::SpectralVector &emissionOut) -> double {
      direction = d5b::normalize(d5b::Vector3(-1, 3, 1));
      distance = d5b::Inf;
      emissionOut.assign(emission);
      return 1;
    };
  }
}

void LumberyardBistro::infiniteLightContributionForEscapedRay(
  d5b::Random &, d5b::Ray, const d5b::SpectralVector &wavelength, d5b::SpectralVector &emission) const {
  for (size_t i = 0; i < wavelength.size(); i++) {
    emission[i] = 2 * mi::normalizedBlackbodyRadiance(9000.0, wavelength[i]);
  }
}
