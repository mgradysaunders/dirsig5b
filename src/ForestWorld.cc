#include "ForestWorld.h"
#include "Camera.h"

#include <Microcosm/Render/ConvertRGB>
#include <Microcosm/Render/Illuminant>
#include <Microcosm/Render/Microsurface>
#include <Microcosm/Render/Prospect>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <filesystem>
#include <iostream>

void ForestWorld::initialize() {
  for (const auto &group : std::filesystem::directory_iterator{"/home/michael/Documents/Models/Broadleaf-Summer"}) {
    std::string groupName = group.path().stem().string();
    auto &plant = plants.emplace_back();
    for (const auto &entry : std::filesystem::directory_iterator{group.path()}) {
      if (entry.path().extension() == ".FBX") {
        std::cout << "Loading ... " << entry.path() << std::endl;
        Assimp::Importer importer;
        if (const auto *scene = importer.ReadFile(entry.path().string(), 0)) {
          uint32_t leafMaterial{0};
          for (size_t i = 0; i < scene->mNumMaterials; i++) {
            auto name = scene->mMaterials[i]->GetName();
            if (std::string_view(name.C_Str()) == "leaf") {
              leafMaterial = i;
              break;
            }
          }
          auto &plantMesh = plant.meshes.emplace_back();
          for (size_t i = 0; i < scene->mNumMeshes; i++) {
            const auto *mesh = scene->mMeshes[i];
            for (size_t j = 0; j < mesh->mNumFaces; j++) {
              const auto &face = mesh->mFaces[j];
              for (size_t k = 1; k + 1 < face.mNumIndices; k++) {
                {
                  const auto &vA = mesh->mVertices[face.mIndices[0]];
                  const auto &vB = mesh->mVertices[face.mIndices[k + 0]];
                  const auto &vC = mesh->mVertices[face.mIndices[k + 1]];
                  plantMesh.positions.push_back(mi::Vector3f(vA.x, vA.y, vA.z));
                  plantMesh.positions.push_back(mi::Vector3f(vB.x, vB.y, vB.z));
                  plantMesh.positions.push_back(mi::Vector3f(vC.x, vC.y, vC.z));
                }
                {
                  const auto &vA = mesh->mTextureCoords[0][face.mIndices[0]];
                  const auto &vB = mesh->mTextureCoords[0][face.mIndices[k + 0]];
                  const auto &vC = mesh->mTextureCoords[0][face.mIndices[k + 1]];
                  plantMesh.texcoords.push_back(mi::Vector2f(vA.x, vA.y));
                  plantMesh.texcoords.push_back(mi::Vector2f(vB.x, vB.y));
                  plantMesh.texcoords.push_back(mi::Vector2f(vC.x, vC.y));
                }
                {
                  const auto &vA = mesh->mNormals[face.mIndices[0]];
                  const auto &vB = mesh->mNormals[face.mIndices[k + 0]];
                  const auto &vC = mesh->mNormals[face.mIndices[k + 1]];
                  plantMesh.normals.push_back(mi::Vector3f(vA.x, vA.y, vA.z));
                  plantMesh.normals.push_back(mi::Vector3f(vB.x, vB.y, vB.z));
                  plantMesh.normals.push_back(mi::Vector3f(vC.x, vC.y, vC.z));
                }
                plantMesh.materials.push_back(mesh->mMaterialIndex == leafMaterial ? 0 : 1);
              }
            }
          }
          plantMesh.build();
        }
      } else if (entry.path().stem().string().find("Alfa") != std::string::npos) {
        plant.leafOpacity = mi::stbi::loadU8(entry.path().string());
      } else if (entry.path().stem().string().find("Ref") != std::string::npos) {
        plant.leafAlbedo = mi::stbi::loadU8(entry.path().string());
      } else if (entry.path().stem().string().find("Bark") != std::string::npos) {
        plant.barkAlbedo = mi::stbi::loadU8(entry.path().string());
      }
    }
    break;
  }

  std::vector<mi::Vector2d> instanceLocations;
  mi::geometry::DynamicKDTree2 instanceLocationTree;
  {
    auto &inst = plantInstances.emplace_back();
    inst.plant = &plants.front();
    inst.mesh = &inst.plant->meshes.front();
  }
  {
    auto &inst = plantInstances.emplace_back();
    inst.plant = &plants.front();
    inst.mesh = &*std::next(inst.plant->meshes.begin());
    inst.transform = mi::DualQuaterniond::translate({200, 0, 0});
  }

  plantInstanceTree.build(plantInstances, 1, [](const auto &inst) -> mi::BoundBox3f {
    mi::BoundBox3f boxResult;
    auto box = inst.mesh->tree[0].box;
    for (auto corner : box.allCorners()) boxResult |= inst.transform.applyForward(d5b::Transform::Rule::Affine, corner);
    return boxResult;
  });
}

std::optional<float>
ForestWorld::PlantInstance::intersect(d5b::Random &random, d5b::Ray ray, d5b::LocalSurface &localSurface) const {
  mi::render::TriangleMesh::Location location;
  if (auto param = mesh->rayTest(mi::Ray3f{ray}, location)) {
    localSurface.position = location.point;
    localSurface.texcoord = location.shading.texcoord;
    localSurface.tangents[0] = location.shading.tangents.col(0);
    localSurface.tangents[1] = location.shading.tangents.col(1);
    localSurface.normal = mi::normalize(location.shading.normal);
    auto texcoord = location.shading.texcoord;
    auto material = mesh->materials[location.index];
    if (material == 0) {
      int y = plant->leafOpacity.size(0) * (1 - mi::fract(texcoord[1]));
      int x = plant->leafOpacity.size(1) * mi::fract(texcoord[0]);
      if (plant->leafOpacity(y, x, 0) == 0) {
        ray.minParam = *param + 1e-3;
        return intersect(random, ray, localSurface);
      }
    }
    localSurface.scatteringProvider = [&, position = localSurface.position, texcoord,
                                       material](const d5b::SpectralVector &wavelength, d5b::Scattering &scattering) {
      if (material == 0) {
        scattering.setLambertDiffuse(1.0, 1.0);
        d5b::SpectralVector Lr{wavelength.shape};
        d5b::SpectralVector Lt{wavelength.shape};
        mi::render::Prospect prospect;
        prospect.chlorophylls = 40;
        // prospect.anthocyanins = mi::lerp(mi::saturate(mi::unlerp(position[2], 100.0, 800.0)), 0, 20);
        // prospect.carotenoids = mi::lerp(mi::saturate(mi::unlerp(position[2], 100.0, 800.0)), 50, 10);
        for (size_t i = 0; i < wavelength.size(); i++) {
          auto [r, t] = prospect(wavelength[i]);
          Lr[i] = r;
          Lt[i] = t;
        }
        int y = plant->leafAlbedo.size(0) * (1 - mi::fract(texcoord[1]));
        int x = plant->leafAlbedo.size(1) * mi::fract(texcoord[0]);
        double a = mi::saturate(mi::decodeSRGB(plant->leafAlbedo(y, x, 0) * (1.0 / 255.0)) * 4);
        Lr *= a;
        Lt *= a;
        scattering.evaluateBSDF = [Lr = std::move(Lr), Lt = std::move(Lt), evaluateBSDF = std::move(scattering.evaluateBSDF)](
                                    d5b::Vector3 omegaO, d5b::Vector3 omegaI, d5b::SpectralVector &f) {
          evaluateBSDF(omegaO, omegaI, f);
          if (mi::signbit(omegaO[2]) == mi::signbit(omegaI[2])) {
            f *= Lr;
            mi::render::DielectricMicrosurface microsurface;
            microsurface.roughness = {0.6, 0.6};
            f += 0.05 * microsurface.singleScatter(omegaO, omegaI).value;
          } else {
            f *= Lt;
          }
        };
      } else {
        auto colorToSpectrum = [&](mi::Vector3d c) {
          d5b::SpectralVector spectrum{wavelength.shape};
          for (size_t i = 0; i < wavelength.size(); i++) {
            spectrum[i] = mi::render::ConvertRGBToAlbedo(c, wavelength[i]);
          }
          return spectrum;
        };
        mi::Vector3d color{1, 1, 1};
        int y = plant->barkAlbedo.size(0) * (1 - mi::fract(texcoord[1]));
        int x = plant->barkAlbedo.size(1) * mi::fract(texcoord[0]);
        color[0] = mi::decodeSRGB(plant->barkAlbedo(y, x, 0) * (1.0 / 255.0));
        color[1] = mi::decodeSRGB(plant->barkAlbedo(y, x, 1) * (1.0 / 255.0));
        color[2] = mi::decodeSRGB(plant->barkAlbedo(y, x, 2) * (1.0 / 255.0));
        scattering.setLambertDiffuse(0.6, 0.0);
        scattering.multiply(colorToSpectrum(color));
      }
    };
    localSurface.withTransform(transform);
    return param;
  }
  return std::nullopt;
}

bool ForestWorld::intersect(d5b::Random &random, d5b::Ray ray, d5b::LocalSurface &localSurface) const {
  bool result{false};
  mi::Ray3f topRay{ray};
  plantInstanceTree.visitRayCast(topRay, [&](const auto &node) {
    const auto &plantInstance = plantInstances[node.first];
    d5b::Ray localRay{ray};
    localRay.withTransform(plantInstance.transform.inverted());
    if (auto param = plantInstance.intersect(random, localRay, localSurface)) {
      ray.maxParam = *param;
      topRay.maxParam = *param;
      result = true;
    }
    return true; // Continue
  });
  if (auto param = mi::Plane3d({0, 0, 1}, 0).rayTest(mi::Ray3d(ray.org, ray.dir, ray.minParam, ray.maxParam))) {
    localSurface.position = ray.org + *param * ray.dir;
    localSurface.tangents[0] = {1, 0, 0};
    localSurface.tangents[1] = {0, 1, 0};
    localSurface.normal = {0, 0, 1};
    localSurface.scatteringProvider = [&](const d5b::SpectralVector &wavelength, d5b::Scattering &scattering) {
      scattering.setLambertDiffuse(0.1, 0);
    };
    result = true;
  }
  return result;
}

void ForestWorld::directLightsForVertex(
  d5b::Random &random,
  const d5b::Vertex &vertex,
  const d5b::SpectralVector &wavelength,
  std::vector<d5b::DirectLight> &directLights) const {
  {
    d5b::SpectralVector emission{wavelength.shape};
    for (size_t i = 0; i < wavelength.size(); i++)
      emission[i] = mi::render::IlluminantD(mi::convertCCTToXY(5004.0f), wavelength[i]);
    auto &sun = directLights.emplace_back();
    sun.importanceSampleSolidAngle = [emission = std::move(emission)](
                                       d5b::Random &random, d5b::Vector3, d5b::Vector3 &direction, double &distance,
                                       d5b::SpectralVector &emissionOut) -> double {
      mi::Vector3d sunDirection = mi::normalize(mi::Vector3d(-1, 3, 1));
      mi::Matrix3d sunBasis = mi::Matrix3d::orthonormalBasis(sunDirection);
      direction = mi::dot(sunBasis, mi::uniformConeSample<double>(0.9999, random));
      distance = d5b::Inf;
      emissionOut.assign(emission);
      return 1; // mi::uniformConePDF<double>(0.999);
    };
  }
}

void ForestWorld::infiniteLightContributionForEscapedRay(
  d5b::Random &random, d5b::Ray ray, const d5b::SpectralVector &wavelength, d5b::SpectralVector &emission) const {
  double z = mi::unlerp(ray.dir[2], -1.0, 1.0);
  for (size_t i = 0; i < wavelength.size(); i++) {
    emission[i] = mi::normalizedBlackbodyRadiance(mi::lerp(z, 3000.0, 12000.0), wavelength[i]);
  }
}

int main() {
  Camera camera;
  camera.basename = "Forest";
  camera.localToWorld = d5b::DualQuaternion::lookAt({1.5 * 1000, 1.5 * 500, 500}, {0, 0, 500}, {0, 0, 1});
  camera.sizeX = 2048; //* 2;
  camera.sizeY = 2048; //* 2;
  camera.fovY = 60.0_degrees;
  camera.dofRadius = 0;
  camera.dofDistance = 600;
  camera.maxBounces = 10;
  camera.maxSamples = 1024;
  camera.initialize();

  ForestWorld forestWorld;
  forestWorld.initialize();

  d5b::Simulation simulation{};
  simulation.sensor = &camera;
  simulation.world = &forestWorld;
  simulation.simulate();
  return 0;
}
