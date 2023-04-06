#include "dirsig5b/Simulation.h"

namespace d5b {

void Simulation::simulate() {
  std::vector<ProblemRequest> problemRequests;
  while (true) {
    problemRequests.clear();
    problemRequests.reserve(512);
    Status status{sensor->recordProblemRequests(problemRequests)};
#pragma omp parallel for
    for (const ProblemRequest &problemRequest : problemRequests) {
      simulate(problemRequest());
    }
    if (status == Status::Done) break;
  }
}

void Simulation::simulate(Problem problem) {
  Random random{problem.seed};
  std::vector<Vertex> path;
  path.reserve(16);
  for (size_t sampleIndex = 0; true; sampleIndex++) {
    path.clear();
    SpectralVector wavelength{problem.wavelength};
    SpectralVector throughput{problem.throughput};
    SpectralVector radiance{wavelength.shape};
    Ray ray{problem.sampleRay(random)};
    ray.medium = world->mediumLookup(ray.org);

    for (size_t bounceIndex = 0; problem.bounceLimit == 0 || bounceIndex < problem.bounceLimit; bounceIndex++) {
      Vertex vertex;
      bool hitSurface = world->intersect(random, ray, vertex.localSurface);
      bool hitVolume = ray.medium && ray.medium->intersect(random, ray, vertex.localVolume);
      if (!hitVolume && !hitSurface) {
        // Escaped!
        break;
      } else if (hitVolume) {
        // Initialize volume vertex.
        vertex.kind = Vertex::Kind::Volume;
        vertex.position = vertex.localVolume.position;
        vertex.localToWorld = vertex.localVolume.localToWorld;

        // If the local volume defines a scattering constructor, call it now.
        if (vertex.localVolume.scatteringProvider) {
          vertex.localVolume.scatteringProvider(wavelength, vertex.scattering.emplace());
        }

      } else if (hitSurface) {
        // Initialize surface vertex.
        vertex.kind = Vertex::Kind::Surface;
        vertex.position = vertex.localSurface.position;
        vertex.localToWorld = vertex.localSurface.localToWorld();

        // If the local surface defines a scattering constructor, call it now.
        if (vertex.localSurface.scatteringProvider) {
          vertex.localSurface.scatteringProvider(wavelength, vertex.scattering.emplace());
        }
      }

      // Set the current throughput.
      vertex.pathThroughput = throughput;

      // Set the path scattering directions. Incident direction is opposite outgoing direction by default.
      vertex.pathOmegaO = -fastNormalize(ray.dir);
      vertex.pathOmegaI = -vertex.pathOmegaO;

      // If the vertex has scattering functions:
      // 1. Sample direct lights, then add to the radiance estimate.
      // 2. Sample scattered direction and accumulate throughput.
      if (vertex.scattering) [[likely]] {
#if 0
        if (estimatorIn.minBounces <= bounces && bounces <= estimatorIn.bounceLimit) {
          for (int i = 0; i < 16; i++) {
            auto lightLoc = light->model->renderMesh.areaSample(random);
            auto lightRay = mi::Ray3f(pathVert.location.point, lightLoc.point - pathVert.location.point, 1e-3f, 0.99f);
            Color value = pathVert.scatter(mi::fastNormalize(lightRay.direction));
            float valuePDF = light->model->renderMesh.solidAnglePDF(pathVert.location.point, lightLoc);
            value /= valuePDF;
            value *= pathVert.throughput / sampleIndex;
            mi::render::TriangleMesh::Location tmpLocation;
            if (mi::isfinite(value).all() && (value > 0).any() && !rayTest(lightRay, tmpLocation))
              color +=
                value * light->emission->emission(lightLoc.shading.localToWorld(), mi::fastNormalize(-lightRay.direction));
          }
        }
#endif
        throughput = 1;
        if (double p = vertex.importanceSample(random, vertex.pathOmegaO, vertex.pathOmegaI, throughput);
            !(p > 0 && mi::isfinite(p)))
          break;
        throughput *= vertex.pathThroughput;
      }

      // Update the ray.
      ray.org = vertex.position;
      ray.dir = vertex.pathOmegaI;
      ray.minParam = 1e-5;
      ray.maxParam = Inf;
      ray.derivatives = {}; // Ignore derivatives after the first bounce.

      // Add the vertex to the path.
      path.emplace_back(std::move(vertex));
    }
    if (problem.acceptPathContribution(path, radiance) == Status::Done) break;
  }
}

} // namespace d5b
