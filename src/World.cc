#include "World.h"

bool World::intersect(d5b::Random &, d5b::Ray ray, d5b::LocalSurface &localSurface) const {
  mi::render::TriangleMesh::Location location;
  if (triangleMesh.rayTest(mi::Ray3f(ray.org, ray.dir, ray.minParam, ray.maxParam), location)) {
    localSurface.position = location.point;
    localSurface.texcoord = location.shading.texcoord;
    localSurface.tangents[0] = location.shading.tangents[0];
    localSurface.tangents[1] = location.shading.tangents[1];
    localSurface.normal = location.shading.normal;
    localSurface.scatteringProvider = [s=mi::fract(localSurface.texcoord[0]),t=mi::fract(localSurface.texcoord[1])](const d5b::SpectralVector &, d5b::Scattering &scattering) {
      scattering.setLambertDiffuse(0.333, 0.0);
      scattering.multiply({1.0, s, t, 1.0});
    };
    return true;
  }
  return false;
}

// TODO Visibility instead of boolean
bool World::intersect(d5b::Random &, d5b::Ray ray) const {
  return triangleMesh.rayTestShadowOnly(mi::Ray3f(ray.org, ray.dir, ray.minParam, ray.maxParam)).has_value();
}

void World::directLightsForVertex(
  d5b::Random &random,
  const d5b::Vertex &vertex,
  const d5b::SpectralVector &wavelength,
  std::vector<d5b::DirectLight> &directLights) const {
  auto &sun = directLights.emplace_back();
  d5b::SpectralVector emissionCurve{wavelength.shape};
  for (size_t i = 0; i < wavelength.size(); i++) {
    emissionCurve[i] = 3 * mi::normalizedBlackbodyRadiance(5000.0, wavelength[i]);
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
