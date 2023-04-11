#include "StatueWorld.h"
#include "Camera.h"

#include <Microcosm/Noise>
#include <Microcosm/Render/Illuminant>
#include <Microcosm/Render/MeasuredConductor>
#include <Microcosm/Render/Microsurface>

void StatueWorld::initialize() {
  auto happy = mi::geometry::Mesh(mi::geometry::FileOBJ("/home/michael/Documents/Models/happy-remesh.obj"));
  happy.calculateNormals();
  // happy.translate(mi::Vector3f(0, 0, -happy.boundBox()[0][2]));
  // happy.scale(4);
  mesh.buildFrom(happy, 2);
}

bool StatueWorld::intersect(d5b::Random &random, d5b::Ray ray, d5b::LocalSurface &localSurface) const {
  bool result{false};
  mi::render::TriangleMesh::Location location;
  if (auto param = mesh.rayTest(mi::Ray3f(ray.org, ray.dir, ray.minParam, ray.maxParam), location)) {
    ray.maxParam = *param;
    localSurface.position = location.point;
    localSurface.texcoord = location.shading.texcoord;
    mi::Matrix3d basis = mi::Matrix3d::orthonormalBasis(mi::normalize(location.shading.normal));
    mi::Noise3d noiseAngle{24};
    basis = mi::dot(mi::Matrix3d(mi::Quaterniond::rotate(6 * mi::abs(noiseAngle(location.point)), basis.col(2))), basis);
    localSurface.normal = basis.col(2);
    localSurface.tangents[0] = basis.col(0);
    localSurface.tangents[1] = basis.col(1);

    mi::Noise3d noise{15};
    mi::Vector3d gradient;
    noise(mi::Vector3d(100, 100, 100) * location.point, &gradient);
    localSurface.normal = mi::normalize(localSurface.normal + 0.05 * gradient);

    mi::Noise3d noiseX{1};
    mi::Noise3d noiseY{7};
    double roughnessX = 0.05; // mi::lerp((0.5 * noiseX(10 * location.point) + 0.5), 0.1, 0.2);
    double roughnessY = 0.8;  // mi::lerp((0.5 * noiseY(10 * location.point) + 0.5), 0.6, 0.9);
    localSurface.scatteringProvider = [this, roughnessX, roughnessY, point = location.point](
                                        const d5b::SpectralVector &wavelength, d5b::Scattering &scattering) {
      scattering = {
        [=](mi::Vector3d omegaO, mi::Vector3d omegaI, d5b::SpectralVector &f) {
          mi::render::MeasuredConductor conductorAl(mi::render::MeasuredConductor::Kind::Al);
          mi::render::MeasuredConductor conductorCu(mi::render::MeasuredConductor::Kind::Au);
          mi::render::ConductiveMicrosurface microsurface;
          microsurface.roughness = {roughnessX, roughnessY};
          if (mi::signbit(omegaO[2]) == mi::signbit(omegaI[2])) {
            for (size_t i = 0; i < wavelength.size(); i++) {
              auto eta = 1.0f / mi::lerp(
                                  float(mi::saturate(mi::unlerp(point[1], 0.5, 1.7))), //
                                  conductorAl.refractiveIndex(wavelength[i]),          //
                                  conductorCu.refractiveIndex(wavelength[i]));
              microsurface.eta.real(eta.real());
              microsurface.eta.imag(eta.imag());
              f[i] = microsurface.singleScatter(omegaO, omegaI).value;
            }
          } else
            f = 0;
        },
        [=](mi::Vector3d omegaO, mi::Vector3d omegaI) -> double {
          mi::render::ConductiveMicrosurface microsurface;
          microsurface.roughness = {roughnessX, roughnessY};
          if (mi::signbit(omegaO[2]) == mi::signbit(omegaI[2]))
            return microsurface.singleScatter(omegaO, omegaI).valuePDF;
          else
            return 0;
        },
        [=](d5b::Random &random, mi::Vector3d omegaO, mi::Vector3d &omegaI, d5b::SpectralVector &beta) -> double {
          mi::render::MeasuredConductor conductorAl(mi::render::MeasuredConductor::Kind::Al);
          mi::render::MeasuredConductor conductorCu(mi::render::MeasuredConductor::Kind::Au);
          mi::render::ConductiveMicrosurface microsurface;
          microsurface.roughness = {roughnessX, roughnessY};
          omegaI = microsurface.singleScatterSample(random, omegaO);
          if (mi::signbit(omegaO[2]) == mi::signbit(omegaI[2])) {
            double p = microsurface.singleScatter(omegaO, omegaI).valuePDF;
            for (size_t i = 0; i < wavelength.size(); i++) {
              auto eta = 1.0f / mi::lerp(
                                  float(mi::saturate(mi::unlerp(point[1], 0.5, 1.7))), //
                                  conductorAl.refractiveIndex(wavelength[i]),          //
                                  conductorCu.refractiveIndex(wavelength[i]));
              microsurface.eta.real(eta.real());
              microsurface.eta.imag(eta.imag());
              beta[i] = microsurface.singleScatter(omegaO, omegaI).value;
            }
            beta *= 1 / p;
            return p;
          } else {
            beta = 0;
            return 0;
          }
        },
      };
    };
    result = true;
  }
  if (auto param = mi::Plane3d({0, 1, 0}, 0).rayTest(mi::Ray3d(ray.org, ray.dir, ray.minParam, ray.maxParam))) {
    localSurface.position = ray.org + *param * ray.dir;
    localSurface.tangents[0] = {0, 0, 1};
    localSurface.tangents[1] = {1, 0, 0};
    localSurface.normal = {0, 1, 0};
    localSurface.scatteringProvider = [&](const d5b::SpectralVector &wavelength, d5b::Scattering &scattering) {
      scattering.setLambertDiffuse(0.1, 0);
    };
    result = true;
  }
  return result;
}

void StatueWorld::directLightsForVertex(
  d5b::Random &random,
  const d5b::Vertex &vertex,
  const d5b::SpectralVector &wavelength,
  std::vector<d5b::DirectLight> &directLights) const {

  {
    d5b::SpectralVector emission{wavelength.shape};
    for (size_t i = 0; i < wavelength.size(); i++) emission[i] = 100 * mi::render::IlluminantF(1, wavelength[i]);
    auto &light1 = directLights.emplace_back();
    light1.importanceSampleSolidAngle = [emission = std::move(emission)](
                                          d5b::Random &random, d5b::Vector3 position, d5b::Vector3 &direction, double &distance,
                                          d5b::SpectralVector &emissionOut) -> double {
      double diskRadius = 1;
      mi::Vector3d diskPosition = {-4, 4, 4};
      mi::Vector3d diskNormal = mi::normalize(diskPosition);
      mi::Matrix3d diskMatrix = mi::Matrix3d::orthonormalBasis(diskNormal);
      mi::Vector2d diskSample = diskRadius * mi::uniformDiskSample<double>(random);
      mi::Vector3d samplePosition = diskPosition + diskMatrix.col(0) * diskSample[0] + diskMatrix.col(1) * diskSample[1];
      direction = mi::fastNormalize(samplePosition - position);
      distance = mi::fastLength(samplePosition - position);
      emissionOut.assign(emission);
      return mi::areaToSolidAngleMeasure<double>(position, samplePosition, diskNormal) /
             (mi::constants::Pi<double> * diskRadius * diskRadius);
    };
    light1.numSubSamples = 2;
  }

  {
    d5b::SpectralVector emission{wavelength.shape};
    for (size_t i = 0; i < wavelength.size(); i++) emission[i] = 0.2 * mi::render::IlluminantF(3, wavelength[i]);
    auto &light2 = directLights.emplace_back();
    light2.importanceSampleSolidAngle = [emission = std::move(emission)](
                                          d5b::Random &random, d5b::Vector3 position, d5b::Vector3 &direction, double &distance,
                                          d5b::SpectralVector &emissionOut) -> double {
      double diskRadius = 5;
      mi::Vector3d diskPosition = {5, 4, -2};
      mi::Vector3d diskNormal = mi::normalize(diskPosition);
      mi::Matrix3d diskMatrix = mi::Matrix3d::orthonormalBasis(diskNormal);
      mi::Vector2d diskSample = diskRadius * mi::uniformDiskSample<double>(random);
      mi::Vector3d samplePosition = diskPosition + diskMatrix.col(0) * diskSample[0] + diskMatrix.col(1) * diskSample[1];
      direction = mi::fastNormalize(samplePosition - position);
      distance = mi::fastLength(samplePosition - position);
      emissionOut.assign(emission);
      return mi::areaToSolidAngleMeasure<double>(position, samplePosition, diskNormal) /
             (mi::constants::Pi<double> * diskRadius * diskRadius);
    };
    light2.numSubSamples = 2;
  }
}

void StatueWorld::infiniteLightContributionForEscapedRay(
  d5b::Random &, d5b::Ray ray, const d5b::SpectralVector &, d5b::SpectralVector &emission) const {
  emission = 2 * mi::max(ray.dir[2], 0);
}

int main() {
  Camera camera;
  camera.localToWorld = d5b::DualQuaternion::lookAt({0, 0.5, 4}, {0, 1.65, 0}, {0, 1, 0});
  camera.sizeX = 1024*4;
  camera.sizeY = 1024*4;
  camera.fovY = 60.0_degrees;
  camera.dofRadius = 0.01;
  camera.dofDistance = 4.5;
  camera.maxBounces = 5;
  camera.maxSamples = 1024;
  camera.gain = 0.5;
  camera.initialize();

  StatueWorld statueWorld;
  statueWorld.initialize();
  d5b::Simulation simulation{};
  simulation.sensor = &camera;
  simulation.world = &statueWorld;
  simulation.simulate();
  return 0;
}
