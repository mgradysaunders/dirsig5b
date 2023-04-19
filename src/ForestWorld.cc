#include "ForestWorld.h"
#include "Camera.h"

#include <Microcosm/Geometry/DynamicKDTree>
#include <Microcosm/Noise>
#include <Microcosm/Render/ConvertRGB>
#include <Microcosm/Render/Illuminant>
#include <Microcosm/Render/Microsurface>
#include <Microcosm/Render/Prospect>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <filesystem>
#include <iostream>

void ForestWorld::initialize() {
  int g = 0;
  for (const auto &group : std::filesystem::directory_iterator{"/home/michael/Documents/Models/Broadleaf-Summer"}) {
    if (g++ > 16) break;
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
      } else if (entry.path().stem().string().find("Ref2") != std::string::npos) {
        plant.leafAlbedo = mi::stbi::loadU8(entry.path().string());
      } else if (entry.path().stem().string().find("Bark") != std::string::npos) {
        plant.barkAlbedo = mi::stbi::loadU8(entry.path().string());
      }
    }
  }

  mi::Pcg32 random;
  for (int i = -7 * 0; i <= +1 * 0; i++)
    for (int j = -6 * 0; j <= +1 * 0; j++) {
      std::vector<mi::Vector2f> instanceLocations;
      {
        mi::geometry::DynamicKDTree2 kdtree;
        int failure = 0;
        while (failure++ < 2048) {
          mi::Vector2f position = {
            (2 * i + 2 * mi::randomize<float>(random) - 1) * 40000, //
            (2 * j + 2 * mi::randomize<float>(random) - 1) * 40000};
          position = mi::uniformDiskSample<float>(random) * 2000;
          if (kdtree.nearest(position).dist > 500) {
            kdtree.insert(position), failure = 0;
            instanceLocations.push_back(position);
          }
        }
      }
      for (const auto &instanceLocation : instanceLocations) {
        auto &inst = plantInstances.emplace_back();
        inst.plant = &*std::next(plants.begin(), random(plants.size()));
        inst.mesh = &*std::next(inst.plant->meshes.begin(), random(inst.plant->meshes.size()));
        inst.transform = mi::DualQuaterniond::translate({instanceLocation[0], instanceLocation[1], 0}) *
                         mi::DualQuaterniond::rotateZ(mi::randomize<double>(random) * 6.28);
        inst.numLayers = mi::randomize<double>(random) * 0.2 + 1.4;
        inst.chlorophylls = mi::lerp(mi::randomize<double>(random), 30, 45);
        inst.anthocyanins = mi::lerp(mi::randomize<double>(random), 0, 5);
        inst.carotenoids = mi::lerp(mi::randomize<double>(random), 2, 8);
        inst.dryMatter = mi::lerp(mi::randomize<double>(random), 0.003, 0.005);
      }
      std::cout << plantInstances.size() << std::endl;
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
    localSurface.withTransform(transform);
    auto texcoord = location.shading.texcoord;
    auto material = mesh->materials[location.index];
    if (material == 0) {
      int y = plant->leafOpacity.size(0) * (1 - mi::fastFract(texcoord[1]));
      int x = plant->leafOpacity.size(1) * mi::fastFract(texcoord[0]);
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
        mi::Noise3d noise;
        mi::render::Prospect prospect;
        double t = mi::saturate(noise(position * 0.00005) * 0.5 + 0.5);
        int y = plant->leafAlbedo.size(0) * (1 - mi::fastFract(texcoord[1]));
        int x = plant->leafAlbedo.size(1) * mi::fastFract(texcoord[0]);
        double a = mi::saturate(mi::decodeSRGB(plant->leafAlbedo(y, x, 0) * (1.0 / 255.0)));
        prospect.numLayers = numLayers;
        prospect.chlorophylls =
          mi::lerp(t, 1, 0.1) * (chlorophylls + noise(0.001 * position) * 5) * mi::lerp(texcoord[1], 1.3, 0.7);
        prospect.anthocyanins = mi::lerp(t, 1, 1.5) * anthocyanins;
        prospect.carotenoids = mi::lerp(t, 1, 2) * carotenoids * mi::lerp(texcoord[1], 0.7, 1.3);
        prospect.dryMatter = dryMatter;
        for (size_t i = 0; i < wavelength.size(); i++) {
          auto [r, t] = prospect(wavelength[i]);
          Lr[i] = r;
          Lt[i] = t;
        }
        Lr *= mi::min(1, mi::lerp(a, 0.25, 1.25));
        Lt *= mi::min(1, mi::lerp(a, 0.25, 1.25));
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
        int y = plant->barkAlbedo.size(0) * (1 - mi::fastFract(texcoord[1]));
        int x = plant->barkAlbedo.size(1) * mi::fastFract(texcoord[0]);
        color[0] = mi::decodeSRGB(plant->barkAlbedo(y, x, 0) * (1.0 / 255.0));
        color[1] = mi::decodeSRGB(plant->barkAlbedo(y, x, 1) * (1.0 / 255.0));
        color[2] = mi::decodeSRGB(plant->barkAlbedo(y, x, 2) * (1.0 / 255.0));
        scattering.setLambertDiffuse(0.8, 0.0);
        scattering.multiply(colorToSpectrum(color));
      }
    };
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
      scattering.setLambertDiffuse(0.04, 0);
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
#if 0
  {
    d5b::SpectralVector emission{wavelength.shape};
    for (size_t i = 0; i < wavelength.size(); i++)
      emission[i] = mi::render::IlluminantD(mi::convertCCTToXY(5004.0f), wavelength[i]);
    auto &sun = directLights.emplace_back();
    sun.importanceSampleSolidAngle = [emission = std::move(emission)](
                                       d5b::Random &random, d5b::Vector3, d5b::Vector3 &direction, double &distance,
                                       d5b::SpectralVector &emissionOut) -> double {
      mi::Vector3d sunDirection = mi::normalize(mi::Vector3d(-1, 3, 2));
      mi::Matrix3d sunBasis = mi::Matrix3d::buildOrthonormalBasis(sunDirection);
      direction = mi::dot(sunBasis, mi::uniformConeSample<double>(0.9999, random));
      distance = d5b::Inf;
      emissionOut.assign(emission);
      return 1; // mi::uniformConePDF<double>(0.999);
    };
  }
#endif

  {
    d5b::SpectralVector emission{wavelength.shape};
    for (size_t i = 0; i < wavelength.size(); i++) emission[i] = 2000 * mi::render::IlluminantF(1, wavelength[i]);
    auto &light1 = directLights.emplace_back();
    light1.importanceSampleSolidAngle = [emission = std::move(emission)](
                                          d5b::Random &random, d5b::Vector3 position, d5b::Vector3 &direction, double &distance,
                                          d5b::SpectralVector &emissionOut) -> double {
      double diskRadius = 200;
      mi::Vector3d diskPosition = {1000, 5000, 7000};
      mi::Vector3d diskNormal = mi::normalize(diskPosition);
      mi::Matrix3d diskMatrix = mi::Matrix3d::buildOrthonormalBasis(diskNormal);
      mi::Vector2d diskSample = diskRadius * mi::uniformDiskSample<double>(random);
      mi::Vector3d samplePosition = diskPosition + diskMatrix.col(0) * diskSample[0] + diskMatrix.col(1) * diskSample[1];
      direction = mi::fastNormalize(samplePosition - position);
      distance = mi::fastLength(samplePosition - position);
      if (dot(direction, diskNormal) > 0)
        emissionOut.assign(emission);
      else
        emissionOut = 0;
      return mi::areaToSolidAngleMeasure<double>(position, samplePosition, diskNormal) /
             (mi::constants::Pi<double> * diskRadius * diskRadius);
    };
    light1.numSubSamples = 2;
  }

  {
    d5b::SpectralVector emission{wavelength.shape};
    for (size_t i = 0; i < wavelength.size(); i++) emission[i] = 500 * mi::render::IlluminantF(5, wavelength[i]);
    auto &light1 = directLights.emplace_back();
    light1.importanceSampleSolidAngle = [emission = std::move(emission)](
                                          d5b::Random &random, d5b::Vector3 position, d5b::Vector3 &direction, double &distance,
                                          d5b::SpectralVector &emissionOut) -> double {
      double diskRadius = 500;
      mi::Vector3d diskPosition = {1000, -5000, 7000};
      mi::Vector3d diskNormal = mi::normalize(diskPosition);
      mi::Matrix3d diskMatrix = mi::Matrix3d::buildOrthonormalBasis(diskNormal);
      mi::Vector2d diskSample = diskRadius * mi::uniformDiskSample<double>(random);
      mi::Vector3d samplePosition = diskPosition + diskMatrix.col(0) * diskSample[0] + diskMatrix.col(1) * diskSample[1];
      direction = mi::fastNormalize(samplePosition - position);
      distance = mi::fastLength(samplePosition - position);
      if (dot(direction, diskNormal) > 0)
        emissionOut.assign(emission);
      else
        emissionOut = 0;
      return mi::areaToSolidAngleMeasure<double>(position, samplePosition, diskNormal) /
             (mi::constants::Pi<double> * diskRadius * diskRadius);
    };
    light1.numSubSamples = 2;
  }
}

void ForestWorld::infiniteLightContributionForEscapedRay(
  d5b::Random &random, d5b::Ray ray, const d5b::SpectralVector &wavelength, d5b::SpectralVector &emission) const {
  // double z = mi::unlerp(ray.dir[2], -1.0, 1.0);
  // for (size_t i = 0; i < wavelength.size(); i++) {
  //   emission[i] = mi::normalizedBlackbodyRadiance(mi::lerp(z, 3000.0, 12000.0), wavelength[i]);
  // }
}

int main() {
  Camera camera;
  camera.basename = "Forest";
  camera.localToWorld = d5b::DualQuaternion::lookAt({0.15 * 20000, 0.15 * 20000, 0.15 * 20000}, {0, 0, 400}, {0, 0, 1});
  camera.sizeX = 1920 * 2;
  camera.sizeY = 1080 * 2;
  camera.fovY = 60.0_degrees;
  camera.dofRadius = 0;
  camera.dofDistance = 600;
  camera.maxBounces = 5;
  camera.maxSamples = 1024;
  camera.numSamplesPerBatch = 8;
  camera.initialize();

  ForestWorld forestWorld;
  forestWorld.initialize();

  d5b::Simulation simulation{};
  simulation.sensor = &camera;
  simulation.world = &forestWorld;
  simulation.simulate();
  return 0;
}
