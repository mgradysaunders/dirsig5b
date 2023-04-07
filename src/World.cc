#include "World.h"

bool World::intersect(d5b::Random &, d5b::Ray ray, d5b::LocalSurface &localSurface) const {
  mi::render::TriangleMesh::Location location;
  if (triangleMesh.rayTest(mi::Ray3f(ray.org, ray.dir, ray.minParam, ray.maxParam), location)) {
    localSurface.position = location.point;
    localSurface.texcoord = location.shading.texcoord;
    localSurface.tangents[0] = location.shading.tangents[0];
    localSurface.tangents[1] = location.shading.tangents[1];
    localSurface.normal = location.shading.normal;
    localSurface.scatteringProvider = [](const d5b::SpectralVector &, d5b::Scattering &scattering) {
      scattering.setLambertian(0.8, 0.0);
    };
    return true;
  }
  return false;
}

void World::directLightsForVertex(
  d5b::Random &random,
  const d5b::Vertex &vertex,
  const d5b::SpectralVector &wavelength,
  std::vector<d5b::DirectLight> &directLights) const {
  auto &sun = directLights.emplace_back();
  sun.sampleSolidAngleEmission = [](
                                   d5b::Random &random, d5b::Vector3, d5b::Vector3 &directionToLight, double &distanceToLight,
                                   d5b::SpectralVector &emission) -> double {
    directionToLight = d5b::normalize(d5b::Vector3(1, 1, 1));
    distanceToLight = d5b::Inf;
    emission.assign(1.0);
    return 1;
  };
}
