#include "StatueWorld.h"
#include "Camera.h"
#include <iostream>

#include <Microcosm/Render/DiffuseModels>
#include <Microcosm/Render/Microsurface>
#include <Microcosm/Render/MicrosurfaceModels>
#include <Microcosm/Render/Primitives>
#include <Microcosm/Render/RefractiveIndex>

void StatueWorld::initialize() {
  // auto happy = mi::geometry::Mesh(mi::geometry::FileOBJ("/home/michael/Documents/Assets/Models/Standalone/dragon1.obj"));
  // happy.rotateZ(-0.6);
  // happy.scale(2);
  // happy.calculateNormals();
  // mesh.build(happy);
}

bool StatueWorld::intersect(d5b::Random &random, d5b::Ray ray, d5b::LocalSurface &localSurface) const {
  bool result{false};
  mi::render::Sphere sphere;
  mi::render::Manifold manifold;
  if (auto param = sphere.intersect(mi::Ray3d(ray), manifold)) {
    ray.maxParam = *param;
#if 0
    mi::SimplexNoise3d noise1{15};
    mi::SimplexNoise3d noise2{18};
    mi::Vector3d gradient1;
    mi::Vector3d gradient2;
    noise1(mi::Vector3d(100, 100, 100) * manifold.proper.point, &gradient1);
    noise2(mi::Vector3d(500, 500, 500) * manifold.proper.point + 30000, &gradient2);
    manifold.pseudo->perturbWithLocalNormal(mi::Vector3d(0, 0, 1) + 0.02 * gradient1 + 0.01 * gradient2);
#endif

    mi::Matrix3d basis = mi::Matrix3d::orthonormalBasis(mi::normalize(manifold.pseudo.normal));
    localSurface.position = manifold.proper.point;
    localSurface.texcoord = manifold.proper.parameters;
    localSurface.normal = basis.col(2);
    localSurface.tangents[0] = basis.col(0);
    localSurface.tangents[1] = basis.col(1);
    localSurface.scatteringProvider =
      [&, point = manifold.proper.point](const d5b::SpectralVector &wavelength, d5b::Scattering &scattering) {
        mi::render::ScatteringMixture mix;
        auto &term1 = mix.mTerms.emplace_back();
        term1.probability = 0.2;
        term1.scattering = mi::render::DisneyDiffuseBRDF(
          mi::render::convertRGBToSpectrumAlbedo(wavelength, {0.1, 0.1, 0.1}),
          mi::render::convertRGBToSpectrumAlbedo(wavelength, {0.1, 0.2, 0.4}),
          mi::render::convertRGBToSpectrumAlbedo(wavelength, {0.2, 0.1, 0.1}),
          mi::render::convertRGBToSpectrumAlbedo(wavelength, {0.1, 0.3, 0.5}));
        auto &term2 = mix.mTerms.emplace_back();
        term2.probability = 0.8;
        term2.scattering = mi::render::DielectricMicrosurfaceBRDF(
          wavelength * 0.0 + 1 / 1.4 /*
        1.0 / mi::render::refractiveIndexOf(mi::render::KnownMetal::Hg)(wavelength) */
          ,
          wavelength * 0.0 + 0.001);
        scattering = std::move(mix);
      };
    result = true;
  }
#if 0
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

#if 0
    mi::Noise3d noise1{15};
    mi::Noise3d noise2{18};
    mi::Vector3d gradient1;
    mi::Vector3d gradient2;
    noise1(mi::Vector3d(100, 100, 100) * location.point, &gradient1);
    noise2(mi::Vector3d(500, 500, 500) * location.point + 30000, &gradient2);
    localSurface.normal = mi::normalize(localSurface.normal + 0.02 * gradient1 + 0.01 * gradient2);
#if 0
    localSurface.scatteringProvider =
      [this, point = location.point](const d5b::SpectralVector &wavelength, d5b::Scattering &scattering) {
        double roughnessX = 0.1;
        double roughnessY = 0.1;
        scattering = {
          [=](mi::Vector3d omegaO, mi::Vector3d omegaI, d5b::SpectralVector &f) {
            mi::render::MeasuredConductor conductorAl(mi::render::MeasuredConductor::Kind::Co);
            mi::render::MeasuredConductor conductorCu(mi::render::MeasuredConductor::Kind::Co);
            mi::render::ConductorMicrosurface microsurface;
            microsurface.roughness = {roughnessX, roughnessY};
            if (mi::signbit(omegaO[2]) == mi::signbit(omegaI[2])) {
              for (size_t i = 0; i < wavelength.size(); i++) {
                auto eta = 1.0f / mi::lerp(
                                    float(mi::saturate(mi::unlerp(point[1], 0.5, 1.7))), //
                                    conductorAl.refractiveIndex(wavelength[i]),          //
                                    conductorCu.refractiveIndex(wavelength[i]));
                microsurface.eta.real(eta.real());
                microsurface.eta.imag(eta.imag());
                double t = mi::unlerp(wavelength[i], 0.4, 0.7);
                microsurface.roughness[0] = mi::lerp(t, 0.03, 0.1);
                microsurface.roughness[1] = mi::lerp(t, 0.1, 0.03);
                f[i] = microsurface.singleScatter(omegaO, omegaI).value;
              }
            } else
              f = 0;
          },
          [=](mi::Vector3d omegaO, mi::Vector3d omegaI) -> double {
            mi::render::ConductorMicrosurface microsurface;
            microsurface.roughness = {roughnessX, roughnessY};
            if (mi::signbit(omegaO[2]) == mi::signbit(omegaI[2]))
              return microsurface.singleScatter(omegaO, omegaI).valuePDF;
            else
              return 0;
          },
          [=](d5b::Random &random, mi::Vector3d omegaO, mi::Vector3d &omegaI, d5b::SpectralVector &beta) -> double {
            mi::render::MeasuredConductor conductorAl(mi::render::MeasuredConductor::Kind::Co);
            mi::render::MeasuredConductor conductorCu(mi::render::MeasuredConductor::Kind::Co);
            mi::render::ConductorMicrosurface microsurface;
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
                double t = mi::unlerp(wavelength[i], 0.4, 0.7);
                microsurface.roughness[0] = mi::lerp(t, 0.03, 0.1);
                microsurface.roughness[1] = mi::lerp(t, 0.1, 0.03);
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
#else
    localSurface.scatteringProvider = [this, point = location.point,
                                       seed = random()](const d5b::SpectralVector &wavelength, d5b::Scattering &scattering) {
      double roughnessX = mi::lerp(mi::saturate(mi::Noise3d()(10 + 4 * point) * 0.5f + 0.5f), 0.1, 0.2);
      double roughnessY = mi::lerp(mi::saturate(mi::Noise3d()(50 + 7 * point) * 0.5f + 0.5f), 0.2, 0.6);
      double thickness = mi::lerp(mi::saturate(mi::Noise3d()(20 * point) * 0.5f + 0.5f), 0.1, 0.6);
      scattering = {
        [=](mi::Vector3d omegaO, mi::Vector3d omegaI, d5b::SpectralVector &f) {
          mi::render::DisneyDiffuseBRDF diff;
          diff.roughness = 1;
          diff.lambert = 0.01;
          mi::render::DielectricMicrosurface brdf{};
          mi::render::microsurface::ThinFilmFresnel fresnel{};
          brdf.roughness = {roughnessX, roughnessY};
          brdf.fresnel = &fresnel;
          brdf.Ft0 = 0;
          brdf.random = d5b::Random(seed);
          fresnel.filmParams.thickness = thickness;
          if (mi::signbit(omegaO[2]) == mi::signbit(omegaI[2])) {
            for (size_t i = 0; i < wavelength.size(); i++) {
              double t = mi::unlerp(wavelength[i], 0.4, 0.7);
              fresnel.filmParams.wavelength = wavelength[i];
              fresnel.filmParams.refractiveIndex = mi::lerp(t, 1.8, 1.45);
              brdf.etaBelow = mi::lerp(mi::sqr(t), 1.4, 1.2);
              diff.roughness = mi::lerp(t, 0.6, 1.0);
              diff.retro = mi::lerp(t, 0.5, 0.08);
              brdf.roughness[1] = mi::lerp(t, 0.8, 1.9) * roughnessY;
              f[i] = brdf.singleScatter(omegaO, omegaI).value + diff.scatter(omegaO, omegaI);
            }
          } else
            f = 0;
        },
        [=](mi::Vector3d omegaO, mi::Vector3d omegaI) -> double {
          mi::render::DielectricMicrosurface brdf{};
          mi::render::microsurface::ThinFilmFresnel fresnel{};
          brdf.roughness = {roughnessX, roughnessY};
          brdf.fresnel = &fresnel;
          brdf.Ft0 = 0;
          brdf.random = d5b::Random(seed);
          fresnel.filmParams.wavelength = 0.5;
          fresnel.filmParams.thickness = thickness;
          if (mi::signbit(omegaO[2]) == mi::signbit(omegaI[2])) {
            return brdf.singleScatter(omegaO, omegaI).valuePDF;
          } else
            return 0;
        },
        [=](d5b::Random &random, mi::Vector3d omegaO, mi::Vector3d &omegaI, d5b::SpectralVector &beta) -> double {
          mi::render::DisneyDiffuseBRDF diff;
          diff.roughness = 1;
          diff.lambert = 0.01;
          mi::render::DielectricMicrosurface brdf{};
          mi::render::microsurface::ThinFilmFresnel fresnel{};
          brdf.roughness = {roughnessX, roughnessY};
          brdf.fresnel = &fresnel;
          brdf.Ft0 = 0;
          brdf.random = d5b::Random(seed);
          fresnel.filmParams.wavelength = 0.5;
          fresnel.filmParams.thickness = thickness;
          if (mi::randomize<double>(random) < 0.3) {
            omegaI = mi::cosineHemisphereSample<double>(random);
            omegaI[2] = mi::sign(omegaO[2]) * omegaI[2];
          } else
            omegaI = brdf.singleScatterSample(random, omegaO);
          if (mi::signbit(omegaO[2]) == mi::signbit(omegaI[2])) {
            double p = 0.7 * brdf.singleScatter(omegaO, omegaI).valuePDF + 0.3 * mi::abs(omegaI[2]) / 3.14159;
            for (size_t i = 0; i < wavelength.size(); i++) {
              double t = mi::unlerp(wavelength[i], 0.4, 0.7);
              fresnel.filmParams.wavelength = wavelength[i];
              fresnel.filmParams.refractiveIndex = mi::lerp(t, 1.8, 1.45);
              brdf.etaBelow = mi::lerp(mi::sqr(t), 1.4, 1.2);
              diff.roughness = mi::lerp(t, 0.6, 1.0);
              diff.retro = mi::lerp(t, 0.5, 0.08);
              brdf.roughness[1] = mi::lerp(t, 0.8, 1.9) * roughnessY;
              beta[i] = brdf.singleScatter(omegaO, omegaI).value + diff.scatter(omegaO, omegaI);
            }
            beta *= 1 / p;
            return p;
          } else {
            beta = 0;
            return 0;
          }
        }};
    };
#endif
#endif
    result = true;
  }
#endif
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

void StatueWorld::directLightsForVertex(
  d5b::Random &random,
  const d5b::Vertex &vertex,
  const d5b::SpectralVector &wavelength,
  std::vector<d5b::DirectLight> &directLights) const {
  {
    d5b::SpectralVector emission = 50 * mi::render::spectrumIlluminantD65(wavelength);
    auto &light1 = directLights.emplace_back();
    light1.importanceSampleSolidAngle = [emission = std::move(emission)](
                                          d5b::Random &random, d5b::Vector3 position, d5b::Vector3 &direction, double &distance,
                                          d5b::SpectralVector &emissionOut) -> double {
      mi::render::Manifold manifold;
      mi::render::TransformedPrimitive disk{mi::DualQuaterniond::lookAt({4, -4, 4}, {0, 0, 0}, {0, 0, 1}), mi::render::Disk{1}};
      double density = disk.solidAngleSample(position, random, manifold);
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
    d5b::SpectralVector emission = 50 * mi::render::spectrumIlluminantF(wavelength, 3);
    auto &light2 = directLights.emplace_back();
    light2.importanceSampleSolidAngle = [emission = std::move(emission)](
                                          d5b::Random &random, d5b::Vector3 position, d5b::Vector3 &direction, double &distance,
                                          d5b::SpectralVector &emissionOut) -> double {
      mi::render::Manifold manifold;
      mi::render::TransformedPrimitive disk{mi::DualQuaterniond::lookAt({-5, 2, 4}, {0, 0, 0}, {0, 0, 1}), mi::render::Disk{1}};
      double density = disk.solidAngleSample(position, random, manifold);
      direction = mi::fastNormalize(manifold.proper.point - position);
      distance = mi::fastLength(manifold.proper.point - position);
      if (dot(direction, manifold.proper.normal) > 0)
        emissionOut.assign(emission);
      else
        emissionOut = 0;
      return density;
    };
    light2.numSubSamples = 2;
  }
}

void StatueWorld::infiniteLightContributionForEscapedRay(
  d5b::Random &, d5b::Ray ray, const d5b::SpectralVector &, d5b::SpectralVector &emission) const {
  // emission = 2 * mi::max(ray.dir[2], 0);
}

int main() {
  Camera camera;
  camera.localToWorld = d5b::DualQuaternion::lookAt({1, -4, 0.5}, {0, 0, 0.5}, {0, 0, 1});
  camera.sizeX = 1920;
  camera.sizeY = 1080;
  camera.fovY = 35.0_degrees;
  camera.wavelengthCount = 20;
  camera.dofRadius = 0; // 0.01;
  camera.dofDistance = 4.5;
  camera.maxBounces = 5;
  camera.maxSamples = 64;
  camera.numSamplesPerBatch = 8;
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
