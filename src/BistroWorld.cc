#include "BistroWorld.h"
#include "Camera.h"

#include <Microcosm/Render/ConvertRGB>
#include <Microcosm/Render/Illuminant>
#include <Microcosm/Render/MeasuredConductor>
#include <Microcosm/Render/Microsurface>
#include <filesystem>
#include <iostream>

void BistroWorld::initialize() {
  fileOBJ = mi::geometry::FileOBJ("/home/michael/Documents/Scenes/lumberyard-bistro/exterior.obj");
  fileMTL = mi::geometry::FileMTL("/home/michael/Documents/Scenes/lumberyard-bistro/exterior.mtl");
  mesh.buildFrom(mi::geometry::Mesh(fileOBJ), 2);
  for (int16_t materialIndex : mesh.materials) {
    if (texturesForMaterial.find(materialIndex) == texturesForMaterial.end()) {
      auto &textures = texturesForMaterial[materialIndex];
      auto &materialName = fileOBJ.metadata.materialNames.at(materialIndex);
      auto &material = fileMTL.materials.at(materialName);
      auto load = [](const std::string &filename) -> mi::stbi::ImageU8 {
        auto path{std::filesystem::path("/home/michael/Documents/Scenes/lumberyard-bistro") / filename};
        std::cout << "Loading " << path.string() << "..." << std::endl;
        return mi::stbi::loadU8(path.string());
      };
      auto name = mi::toLower(materialName);
      if (name.find("metal") != std::string::npos) {
        textures.kind = MaterialKind::Metal;
      } else if (name.find("fabric") != std::string::npos) {
        textures.kind = MaterialKind::Cloth;
      } else if (
        name.find("pavement") != std::string::npos || name.find("stone") != std::string::npos ||
        name.find("concrete") != std::string::npos) {
        textures.kind = MaterialKind::Pavement;
      } else if (
        name.find("leaf") != std::string::npos || name.find("leaves") != std::string::npos ||
        name.find("flower") != std::string::npos) {
        textures.kind = MaterialKind::Leaf;
      } else if (name.find("wood") != std::string::npos) {
        textures.kind = MaterialKind::Wood;
      }
      if (material.ambientTexture) {
        textures.albedo = load(*material.ambientTexture);
      } else if (material.diffuseTexture) {
        textures.albedo = load(*material.diffuseTexture);
      }
      if (material.bumpTexture) textures.normal = load(*material.bumpTexture);
      if (material.opacityTexture) textures.opacity = load(*material.opacityTexture);
    }
  }
}

bool BistroWorld::intersect(d5b::Random &random, d5b::Ray ray, d5b::LocalSurface &localSurface) const {
  mi::render::TriangleMesh::Location location;
  if (auto param = mesh.rayTest(mi::Ray3f(ray.org, ray.dir, ray.minParam, ray.maxParam), location)) {
    auto materialIndex = mesh.materials.at(location.index);
    auto &material = fileMTL.materials.at(fileOBJ.metadata.materialNames.at(materialIndex));
    localSurface.position = location.point;
    localSurface.texcoord = location.shading.texcoord;
    localSurface.tangents[0] = location.shading.tangents.col(0);
    localSurface.tangents[1] = location.shading.tangents.col(1);
    localSurface.normal = mi::normalize(location.shading.normal);
    auto texcoord = location.shading.texcoord;
    auto itr = texturesForMaterial.find(materialIndex);
    if (itr != texturesForMaterial.end() && itr->second.normal) {
      auto &normal = *itr->second.normal;
      int y = normal.size(0) * (1 - mi::fastFract(texcoord[1]));
      int x = normal.size(1) * mi::fastFract(texcoord[0]);
      mi::Vector3d localNormal = {
        normal(y, x, 0) * (1.0 / 255.0) * 2 - 1, //
        normal(y, x, 1) * (1.0 / 255.0) * 2 - 1, //
        normal(y, x, 2) * (1.0 / 255.0) * 2 - 1};
      localSurface.normal = mi::normalize(mi::dot(localSurface.localToWorld(), localNormal));
    }
    mi::Vector4d color{1, 1, 1, 1};
    if (itr != texturesForMaterial.end() && itr->second.albedo) {
      auto &albedo = *itr->second.albedo;
      int y = albedo.size(0) * (1 - mi::fastFract(texcoord[1]));
      int x = albedo.size(1) * mi::fastFract(texcoord[0]);
      color[0] = mi::decodeSRGB(albedo(y, x, 0) * (1.0 / 255.0));
      color[1] = mi::decodeSRGB(albedo(y, x, 1) * (1.0 / 255.0));
      color[2] = mi::decodeSRGB(albedo(y, x, 2) * (1.0 / 255.0));
      color[3] = 1;
    }
#define DIRSIG_STYLE 0
#if !DIRSIG_STYLE
    if (itr != texturesForMaterial.end() && itr->second.opacity) {
      auto &opacity = *itr->second.opacity;
      int y = opacity.size(0) * (1 - mi::fastFract(texcoord[1]));
      int x = opacity.size(1) * mi::fastFract(texcoord[0]);
      if (opacity(y, x, 0) == 0) {
        ray.minParam = *param + 1e-3;
        return intersect(random, ray, localSurface);
      }
    }
#endif
    MaterialKind kind = MaterialKind::Default;
    if (itr != texturesForMaterial.end()) kind = itr->second.kind;
    localSurface.scatteringProvider = [&, kind, color](const d5b::SpectralVector &wavelength, d5b::Scattering &scattering) {
      auto colorToSpectrum = [&](mi::Vector3d c) {
        d5b::SpectralVector spectrum{wavelength.shape};
        for (size_t i = 0; i < wavelength.size(); i++) {
#if DIRSIG_STYLE
          spectrum[i] = 0;
          if (0.6 <= wavelength[i] && wavelength[i] < 0.7) spectrum[i] += mi::encodeSRGB(c[0]);
          if (0.5 <= wavelength[i] && wavelength[i] < 0.6) spectrum[i] += mi::encodeSRGB(c[1]);
          if (0.4 <= wavelength[i] && wavelength[i] < 0.5) spectrum[i] += mi::encodeSRGB(c[2]);
#else
          spectrum[i] = mi::render::ConvertRGBToAlbedo(c, wavelength[i]);
#endif
        }
        return spectrum;
      };
#if DIRSIG_STYLE
      d5b::Scattering diffuse;
      diffuse.setLambertDiffuse(0.8, 0);
      diffuse.multiply(colorToSpectrum(color));
      scattering = std::move(diffuse);
#else
      if (kind == MaterialKind::Metal) {
        scattering = {
          [=](mi::Vector3d omegaO, mi::Vector3d omegaI, d5b::SpectralVector &f) {
            mi::render::MeasuredConductor conductor(mi::render::MeasuredConductor::Kind::CuZn);
            mi::render::ConductorMicrosurface microsurface;
            microsurface.roughness = {0.15, 0.15};
            if (mi::signbit(omegaO[2]) == mi::signbit(omegaI[2])) {
              for (size_t i = 0; i < wavelength.size(); i++) {
                auto eta = 1.0f / conductor.refractiveIndex(wavelength[i]);
                microsurface.eta.real(eta.real());
                microsurface.eta.imag(eta.imag());
                f[i] = microsurface.singleScatter(omegaO, omegaI).value;
              }
            } else
              f = 0;
          },
          [=](mi::Vector3d omegaO, mi::Vector3d omegaI) -> double {
            mi::render::ConductorMicrosurface microsurface;
            microsurface.roughness = {0.15, 0.15};
            if (mi::signbit(omegaO[2]) == mi::signbit(omegaI[2]))
              return microsurface.singleScatter(omegaO, omegaI).valuePDF;
            else
              return 0;
          },
          [=](d5b::Random &random, mi::Vector3d omegaO, mi::Vector3d &omegaI, d5b::SpectralVector &beta) -> double {
            mi::render::ConductorMicrosurface microsurface;
            microsurface.roughness = {0.15, 0.15};
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
        scattering.multiply(colorToSpectrum(color) * 3);
      } else {
        double roughness{0.2};
        d5b::Scattering diffuse;
        if (kind == MaterialKind::Leaf) {
          diffuse.setLambertDiffuse(0.5, 0.5);
          roughness = 0.1;
        } else if (kind == MaterialKind::Cloth) {
          diffuse.setDisneyDiffuse(0.4, 1.0, 0.2, 0.5);
          roughness = 0.5;
        } else if (kind == MaterialKind::Wood) {
          diffuse.setDisneyDiffuse(0.7, 1.0, 0.5, 0.0);
          roughness = 0.3;
        } else if (kind == MaterialKind::Pavement) {
          diffuse.setDisneyDiffuse(0.5, 1.0, 0.1, 0.0);
          roughness = 0.15;
        } else {
          diffuse.setDisneyDiffuse(0.3, 1.0, 0.2, 0.0);
        }
        diffuse.multiply(colorToSpectrum(color));
        mi::render::DielectricMicrosurface microsurface;
        microsurface.roughness = {roughness, roughness};
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
#endif
    };
    return true;
  }
  return false;
}

void BistroWorld::directLightsForVertex(
  d5b::Random &random,
  const d5b::Vertex &vertex,
  const d5b::SpectralVector &wavelength,
  std::vector<d5b::DirectLight> &directLights) const {

  {
    d5b::SpectralVector emission{wavelength.shape};
    for (size_t i = 0; i < wavelength.size(); i++)
      emission[i] = 0.8 * mi::render::IlluminantD(mi::convertCCTToXY(5004.0f), wavelength[i]);
    auto &sun = directLights.emplace_back();
    sun.importanceSampleSolidAngle = [emission = std::move(emission)](
                                       d5b::Random &random, d5b::Vector3, d5b::Vector3 &direction, double &distance,
                                       d5b::SpectralVector &emissionOut) -> double {
      mi::Vector3d sunDirection = mi::normalize(mi::Vector3d(-1, 3, 1));
      mi::Matrix3d sunBasis = mi::Matrix3d::buildOrthonormalBasis(sunDirection);
      direction = mi::dot(sunBasis, mi::uniformConeSample<double>(0.9999, random));
      distance = d5b::Inf;
      emissionOut.assign(emission);
      return 1; // mi::uniformConePDF<double>(0.999);
    };
  }
}

void BistroWorld::infiniteLightContributionForEscapedRay(
  d5b::Random &, d5b::Ray, const d5b::SpectralVector &wavelength, d5b::SpectralVector &emission) const {
  for (size_t i = 0; i < wavelength.size(); i++) {
    emission[i] = 8 * mi::normalizedBlackbodyRadiance(12000.0, wavelength[i]);
  }
}

int main() {
  Camera camera;
  camera.localToWorld = d5b::DualQuaternion::lookAt({100, 155, 525}, {-20, 145, 0}, {0, 1, 0});
  camera.sizeX = 1920 * 2;
  camera.sizeY = 1080 * 2;
  camera.fovY = 75.0_degrees;
  camera.dofRadius = 1;
  camera.dofDistance = 35;
  camera.maxBounces = 5;
  camera.maxSamples = 1024;
  camera.initialize();

  BistroWorld bistroWorld;
  bistroWorld.initialize();
  d5b::Simulation simulation{};
  simulation.sensor = &camera;
  simulation.world = &bistroWorld;
  simulation.simulate();
  return 0;
}
