#include "ForestWorld.h"
#include "Camera.h"

#include <Microcosm/Geometry/DynamicKDTree>
#include <Microcosm/Noise>
#include <Microcosm/Render/DiffuseModels>
#include <Microcosm/Render/Primitives>
#include <Microcosm/Render/Prospect>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <filesystem>
#include <iostream>

void ForestWorld::initialize() {
  int g = 0;
  for (const auto &group : std::filesystem::directory_iterator{"/home/michael/Documents/Assets/Models/Broadleaf-Summer"}) {
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
          std::vector<mi::Vector3f> positions;
          std::vector<mi::Vector2f> texcoords;
          std::vector<mi::Vector3f> normals;
          std::vector<int16_t> materials;
          for (size_t i = 0; i < scene->mNumMeshes; i++) {
            const auto *mesh = scene->mMeshes[i];
            for (size_t j = 0; j < mesh->mNumFaces; j++) {
              const auto &face = mesh->mFaces[j];
              for (size_t k = 1; k + 1 < face.mNumIndices; k++) {
                {
                  const auto &vA = mesh->mVertices[face.mIndices[0]];
                  const auto &vB = mesh->mVertices[face.mIndices[k + 0]];
                  const auto &vC = mesh->mVertices[face.mIndices[k + 1]];
                  positions.push_back(mi::Vector3f(vA.x, vA.y, vA.z));
                  positions.push_back(mi::Vector3f(vB.x, vB.y, vB.z));
                  positions.push_back(mi::Vector3f(vC.x, vC.y, vC.z));
                }
                {
                  const auto &vA = mesh->mTextureCoords[0][face.mIndices[0]];
                  const auto &vB = mesh->mTextureCoords[0][face.mIndices[k + 0]];
                  const auto &vC = mesh->mTextureCoords[0][face.mIndices[k + 1]];
                  texcoords.push_back(mi::Vector2f(vA.x, vA.y));
                  texcoords.push_back(mi::Vector2f(vB.x, vB.y));
                  texcoords.push_back(mi::Vector2f(vC.x, vC.y));
                }
                {
                  const auto &vA = mesh->mNormals[face.mIndices[0]];
                  const auto &vB = mesh->mNormals[face.mIndices[k + 0]];
                  const auto &vC = mesh->mNormals[face.mIndices[k + 1]];
                  normals.push_back(mi::Vector3f(vA.x, vA.y, vA.z));
                  normals.push_back(mi::Vector3f(vB.x, vB.y, vB.z));
                  normals.push_back(mi::Vector3f(vC.x, vC.y, vC.z));
                }
                materials.push_back(mesh->mMaterialIndex == leafMaterial ? 0 : 1);
              }
            }
          }
          plantMesh.positions.resize(positions.size());
          std::copy(&positions[0][0], &positions.back()[0] + 3, &plantMesh.positions(0, 0));
          if (!texcoords.empty()) {
            plantMesh.texcoords.emplace();
            plantMesh.texcoords->resize(texcoords.size());
            std::copy(&texcoords[0][0], &texcoords.back()[0] + 2, &(*plantMesh.texcoords)(0, 0));
          }
          if (!normals.empty()) {
            plantMesh.normals.emplace();
            plantMesh.normals->resize(normals.size());
            std::copy(&normals[0][0], &normals.back()[0] + 3, &(*plantMesh.normals)(0, 0));
          }
          if (!materials.empty()) {
            plantMesh.materials.emplace();
            plantMesh.materials->resize(materials.size());
            std::copy(materials.begin(), materials.end(), &(*plantMesh.materials)[0]);
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
  for (int i = -7; i <= +1; i++)
    for (int j = -6; j <= +1; j++) {
      std::vector<mi::Vector2f> instanceLocations;
      {
        mi::geometry::DynamicKDTree2 kdtree;
        int failure = 0;
        while (failure++ < 2048) {
          mi::Vector2f position = {
            (2 * i + 2 * mi::randomize<float>(random) - 1) * 40000, //
            (2 * j + 2 * mi::randomize<float>(random) - 1) * 40000};
          /* position = mi::render::uniformDiskSample(random) * 2000; */
          if (kdtree.nearestTo(position).dist > 500) {
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

  plantInstanceTree.build(1, plantInstances, [](const auto &inst) -> mi::BoundBox3f {
    mi::BoundBox3f boxResult;
    auto box = inst.mesh->triangleBVH[0].box;
    for (auto corner : box.allCorners()) boxResult |= inst.transform.applyForward(d5b::Transform::Rule::Affine, corner);
    return boxResult;
  });
}

std::optional<float>
ForestWorld::PlantInstance::intersect(d5b::Random &random, d5b::Ray ray, d5b::LocalSurface &localSurface) const {
  mi::render::Manifold manifold;
  if (auto param = mesh->intersect(mi::Ray3d(ray), manifold)) {
    ray.maxParam = *param;
    mi::Matrix3d basis = manifold.pseudo.orthonormalLocalToWorld();
    localSurface.position = manifold.proper.point;
    localSurface.texcoord = manifold.pseudo.parameters;
    localSurface.normal = basis.col(2);
    localSurface.tangents[0] = basis.col(0);
    localSurface.tangents[1] = basis.col(1);
    localSurface.withTransform(transform);
    auto material = (*mesh->materials)[manifold.primitiveIndex];
    if (material == 0) {
      int y = plant->leafOpacity.size(0) * (1 - mi::fastFract(localSurface.texcoord[1]));
      int x = plant->leafOpacity.size(1) * mi::fastFract(localSurface.texcoord[0]);
      if (plant->leafOpacity(y, x, 0) == 0) {
        ray.minParam = *param + 1e-3;
        return intersect(random, ray, localSurface);
      }
    }
    localSurface.scatteringProvider = [this, point = manifold.proper.point, material, texcoord = localSurface.texcoord](
                                        const d5b::SpectralVector &wavelength, d5b::Scattering &scattering) {
      if (material == 0) {
        mi::render::Prospect prospect;
        prospect.numLayers = numLayers;
        prospect.chlorophylls = chlorophylls;
        prospect.anthocyanins = anthocyanins;
        prospect.carotenoids = carotenoids;
        prospect.dryMatter = dryMatter;
        auto [totalR, totalT] = prospect(wavelength);
        // int y = plant->leafAlbedo.size(0) * (1 - mi::fastFract(texcoord[1]));
        // int x = plant->leafAlbedo.size(1) * mi::fastFract(texcoord[0]);
        // double a = mi::saturate(mi::decodeSRGB(plant->leafAlbedo(y, x, 0) * (1.0 / 255.0)));
        scattering = mi::render::LambertBSDF(totalR, totalT);
      } else {
        mi::Vector3d color{1, 1, 1};
        int y = plant->barkAlbedo.size(0) * (1 - mi::fastFract(texcoord[1]));
        int x = plant->barkAlbedo.size(1) * mi::fastFract(texcoord[0]);
        color[0] = mi::decodeSRGB(plant->barkAlbedo(y, x, 0) * (1.0 / 255.0));
        color[1] = mi::decodeSRGB(plant->barkAlbedo(y, x, 1) * (1.0 / 255.0));
        color[2] = mi::decodeSRGB(plant->barkAlbedo(y, x, 2) * (1.0 / 255.0));
        scattering = mi::render::LambertBSDF(mi::render::convertRGBToSpectrumAlbedo(wavelength, color), wavelength * 0.0);
      }
    };
    return *param;
  }
  return {};
#if 0
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
#endif
}

bool ForestWorld::intersect(d5b::Random &random, d5b::Ray ray, d5b::LocalSurface &localSurface) const {
  bool result{false};
  mi::Ray3f topRay{ray};
  plantInstanceTree.visitRayCast(topRay, [&](const auto &node) {
    const auto &plantInstance = plantInstances[node.first];
    d5b::Ray localRay{ray};
    localRay.withTransform(plantInstance.transform.inverted());
    if (auto param = plantInstance.intersect(random, localRay, localSurface)) {
      ray.maxParam = *param, topRay.maxParam = *param;
      result = true;
    }
    return true; // Continue
  });
  if (auto param = mi::Plane3d({0, 0, 1}, 0).rayCast(mi::Ray3d(ray))) {
    localSurface.position = ray.org + *param * ray.dir;
    localSurface.tangents[0] = {1, 0, 0};
    localSurface.tangents[1] = {0, 1, 0};
    localSurface.normal = {0, 0, 1};
    localSurface.scatteringProvider = [&](const d5b::SpectralVector &wavelength, d5b::Scattering &scattering) {
      scattering = mi::render::LambertBSDF(wavelength * 0.0 + 0.1, wavelength * 0.0);
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
#if 1
  {
    d5b::SpectralVector emission = mi::render::spectrumIlluminantD50(wavelength);
    auto &sun = directLights.emplace_back();
    sun.importanceSampleSolidAngle = [emission = std::move(emission)](
                                       d5b::Random &random, d5b::Vector3, d5b::Vector3 &direction, double &distance,
                                       d5b::SpectralVector &emissionOut) -> double {
      mi::Vector3d sunDirection = mi::normalize(mi::Vector3d(-1, 3, 2));
      mi::Matrix3d sunBasis = mi::Matrix3d::orthonormalBasis(sunDirection);
      direction = mi::dot(sunBasis, mi::render::uniformConeSample(random, 0.9999));
      distance = d5b::Inf;
      emissionOut.assign(emission);
      return 1; // mi::uniformConePDF<double>(0.999);
    };
  }
#endif

#if 0
  {
    d5b::SpectralVector emission = 2000 * mi::render::spectrumIlluminantF(wavelength, 1);
    auto &light1 = directLights.emplace_back();
    light1.importanceSampleSolidAngle = [emission = std::move(emission)](
                                          d5b::Random &random, d5b::Vector3 position, d5b::Vector3 &direction, double &distance,
                                          d5b::SpectralVector &emissionOut) -> double {
      mi::render::Manifold manifold;
      mi::render::Disk disk{200};
      mi::render::TransformedPrimitive diskInstance{
        mi::DualQuaterniond::lookAt({1000, 5000, 7000}, {0, 0, 0}, {0, 0, 1}), &disk};
      double density = diskInstance.solidAngleSample(position, random, manifold);
      direction = mi::fastNormalize(manifold.proper.point - position);
      distance = mi::fastLength(manifold.proper.point - position);
      if (dot(direction, manifold.proper.normal) > 0)
        emissionOut.assign(emission);
      else
        emissionOut = 0;
      return density;
    };
    light1.numSubSamples = 2;
  }

  {
    d5b::SpectralVector emission = 500 * mi::render::spectrumIlluminantF(wavelength, 5);
    auto &light1 = directLights.emplace_back();
    light1.importanceSampleSolidAngle = [emission = std::move(emission)](
                                          d5b::Random &random, d5b::Vector3 position, d5b::Vector3 &direction, double &distance,
                                          d5b::SpectralVector &emissionOut) -> double {
      mi::render::Manifold manifold;
      mi::render::Disk disk{500};
      mi::render::TransformedPrimitive diskInstance{
        mi::DualQuaterniond::lookAt({1000, -5000, 7000}, {0, 0, 0}, {0, 0, 1}), &disk};
      double density = diskInstance.solidAngleSample(position, random, manifold);
      direction = mi::fastNormalize(manifold.proper.point - position);
      distance = mi::fastLength(manifold.proper.point - position);
      if (dot(direction, manifold.proper.normal) > 0)
        emissionOut.assign(emission);
      else
        emissionOut = 0;
      return density;
    };
    light1.numSubSamples = 2;
  }
#endif
}

void ForestWorld::infiniteLightContributionForEscapedRay(
  d5b::Random &random, d5b::Ray ray, const d5b::SpectralVector &wavelength, d5b::SpectralVector &emission) const {
  double z = mi::unlerp(ray.dir[2], -1.0, 1.0);
  emission = mi::render::spectrumBlackbody(wavelength, mi::lerp(z, 3000.0, 12000.0));
}

int main() {
  Camera camera;
  camera.basename = "Forest";
  camera.localToWorld = d5b::DualQuaternion::lookAt({20000, 20000, 20000}, {0, 0, 400}, {0, 0, 1});
  camera.sizeX = 1920;
  camera.sizeY = 1080;
  camera.fovY = 60.0_degrees;
  camera.wavelengthCount = 200;
  camera.dofRadius = 0;
  camera.dofDistance = 600;
  camera.maxBounces = 5;
  camera.maxSamples = 32;
  camera.numSamplesPerBatch = 2;
  camera.gain = 2;

  camera.initialize();

  ForestWorld forestWorld;
  forestWorld.initialize();

  d5b::Simulation simulation{};
  simulation.sensor = &camera;
  simulation.world = &forestWorld;
  simulation.simulate();
  return 0;
}
