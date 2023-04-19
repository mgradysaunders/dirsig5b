#include "FractalWorld.h"
#include "Camera.h"

#include <Microcosm/Noise>
#include <Microcosm/Render/ConvertRGB>
#include <Microcosm/Render/DiffuseModels>
#include <Microcosm/Render/Illuminant>
#include <Microcosm/Render/MeasuredConductor>
#include <Microcosm/Render/Microsurface>
#include <Microcosm/Render/MicrosurfaceModels>
#include <Microcosm/Render/Noise>
#include <iostream>

void FractalWorld::initialize() {}

bool FractalWorld::intersect(d5b::Random &random, d5b::Ray ray, d5b::LocalSurface &localSurface) const {
  if (auto param = mi::Plane3d({0, 0, 1}, 0).rayTest(mi::Ray3d(ray.org, ray.dir, ray.minParam, ray.maxParam))) {
    mi::render::Noise<double(mi::Vector2d, mi::Vector2d *)> noise = [](mi::Vector2d x, mi::Vector2d *d) -> double {
      double v = 0.02 * mi::Noise2d()(5 * x, d);
      if (d) *d *= 0.02 * 5;
      return v;
    };
    noise = mi::render::musgraveRidgedMultifractal(noise);
    noise -= [](mi::Vector2d x, mi::Vector2d *d) -> double {
      if (d) *d = 0;
      return 0.75;
    };
    double t0 = *param;
    double t1 = ray.maxParam;
    mi::Vector3d positionPrev;
    mi::Vector3d positionCurr;
    if (ray.org[2] < 0) {
      t0 = ray.minParam;
      t1 = *param;
    }
    int i = 0;
    double h0 = noise(mi::Vector2d(ray.org));
    double t = t0;
    while (t < t1 && i++ < 100) {
      mi::Vector3d position = ray.org + t * ray.dir;
      mi::Vector2d d;
      double h = noise(mi::Vector2d(position), &d);
      if (position[2] > 1e-3) return false;
      if (h > position[2]) {
        localSurface.position = position;
        localSurface.tangents[0] = {1, 0, 0};
        localSurface.tangents[1] = {0, 1, 0};
        if (h > 0)
          localSurface.normal = {0, 0, 1};
        else
          localSurface.normal = mi::normalize(mi::Vector3d(-d[0], -d[1], 1));
        localSurface.position += 2e-3 * localSurface.normal;
        localSurface.scatteringProvider = [&](const d5b::SpectralVector &, d5b::Scattering &scattering) {
          scattering.setLambertDiffuse(0.1, 0);
        };
        return true;
      }
      t += 1e-3;
    }
  }
  return false;
}

void FractalWorld::directLightsForVertex(
  d5b::Random &random,
  const d5b::Vertex &vertex,
  const d5b::SpectralVector &wavelength,
  std::vector<d5b::DirectLight> &directLights) const {

  {
    d5b::SpectralVector emission{wavelength.shape};
    for (size_t i = 0; i < wavelength.size(); i++) emission[i] = 160 * mi::render::IlluminantF(1, wavelength[i]);
    auto &light1 = directLights.emplace_back();
    light1.importanceSampleSolidAngle = [emission = std::move(emission)](
                                          d5b::Random &random, d5b::Vector3 position, d5b::Vector3 &direction, double &distance,
                                          d5b::SpectralVector &emissionOut) -> double {
      double diskRadius = 1;
      mi::Vector3d diskPosition = {4, -4, 4};
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
    for (size_t i = 0; i < wavelength.size(); i++) emission[i] = 50 * mi::render::IlluminantF(3, wavelength[i]);
    auto &light2 = directLights.emplace_back();
    light2.importanceSampleSolidAngle = [emission = std::move(emission)](
                                          d5b::Random &random, d5b::Vector3 position, d5b::Vector3 &direction, double &distance,
                                          d5b::SpectralVector &emissionOut) -> double {
      double diskRadius = 1;
      mi::Vector3d diskPosition = {-5, 2, 4};
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
    light2.numSubSamples = 2;
  }
}

void FractalWorld::infiniteLightContributionForEscapedRay(
  d5b::Random &, d5b::Ray ray, const d5b::SpectralVector &, d5b::SpectralVector &emission) const {
  // emission = 2 * mi::max(ray.dir[2], 0);
}

int main() {
  Camera camera;
  camera.localToWorld = d5b::DualQuaternion::lookAt({1, -4, 0.5}, {0, 0, 0.5}, {0, 0, 1});
  camera.sizeX = 1920;
  camera.sizeY = 1080;
  camera.fovY = 35.0_degrees;
  camera.dofRadius = 0; // 0.01;
  camera.dofDistance = 4.5;
  camera.maxBounces = 3;
  camera.maxSamples = 1024;
  camera.numSamplesPerBatch = 1;
  camera.gain = 0.5;
  camera.initialize();

  FractalWorld fractalWorld;
  fractalWorld.initialize();
  d5b::Simulation simulation{};
  simulation.sensor = &camera;
  simulation.world = &fractalWorld;
  simulation.simulate();
  return 0;
}
