#include "dirsig5b/Simulation.h"

#include <iostream>
#include <omp.h>

namespace d5b {

void Simulation::simulate() {
  std::vector<ProblemRequest> problemRequests;
  while (true) {
    problemRequests.clear();
    problemRequests.reserve(512);
    sensor->request(problemRequests);
#pragma omp parallel for num_threads(100) schedule(dynamic)
    for (const auto &problemRequest : problemRequests) {
      simulate(problemRequest());
    }
    if (sensor->finish() == Status::Done) break;
  }
}

void Simulation::simulate(Problem problem) {
  Random random{problem.seed};
  std::vector<DirectLight> directLights;
  directLights.reserve(16);
  std::vector<Vertex> path;
  path.reserve(16);
  SpectralVector wavelength{problem.wavelength};
  SpectralVector throughput{problem.throughput};
  SpectralVector radiance{wavelength.shape};
  SpectralVector emission{wavelength.shape};
  SpectralVector transmission{wavelength.shape};
  SpectralVector direct{wavelength.shape};
  for (size_t sampleIndex = 0; true; sampleIndex++) {
    path.clear();
    Ray ray{problem.sampleRay(random)};
    ray.medium = world->mediumLookup(ray.org);

    // Reset things.
    throughput.assign(problem.throughput);
    radiance = 0;
    emission = 0;
    transmission = 0;
    direct = 0;

    for (size_t bounceIndex = 0; problem.maxBounces == 0 || bounceIndex < problem.maxBounces; bounceIndex++) {
      Vertex vertex;
      bool hitSurface = world->intersect(random, ray, vertex.localSurface);
      bool hitVolume = ray.medium && ray.medium->intersect(random, ray, vertex.localVolume);
      if (!hitVolume && !hitSurface) {
        emission = 0;
        world->infiniteLightContributionForEscapedRay(random, ray, wavelength, emission);
        radiance += throughput * emission;
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

        // Sample direct lights for the vertex.
        directLights.clear(), world->directLightsForVertex(random, vertex, wavelength, directLights);
        for (const auto &directLight : directLights) {
          for (size_t subSample = 0; subSample < directLight.numSubSamples; subSample++) {
            // Invoke the light solid-angle sampling routine.
            Vector3 omegaO{vertex.pathOmegaO};
            Vector3 omegaI{0};
            double shadowDistance{0};
            double solidAngleDensity{
              directLight.numSubSamples *
              directLight.importanceSampleSolidAngle(random, vertex.position, omegaI, shadowDistance, emission)};
            if (isFiniteAndPositive(solidAngleDensity)) {
              // The probability density of the sample is legitimate, so evaluate the BSDF and account for
              // all terms except for transmission/shadowing. There is a chance the BSDF evaluates to zero
              // due to reflection/transmission mismatch at the surface, which would allow us to skip
              // tracing the shadow ray, which is almost always the most expensive.
              vertex.evaluateBSDF(omegaO, omegaI, direct);
              direct *= (1 / solidAngleDensity) * emission * throughput;
              if (isFiniteAndPositive(direct)) {
                // The solid-angle density combined with the emission, BSDF, and current path throughput
                // is still legitimate and non-zero. Now construct and trace the shadow ray, then add the
                // result to the current path radiance.
                transmission = 1;
                Ray shadowRay{ray};
                shadowRay.org = vertex.position;
                shadowRay.dir = omegaI;
                shadowRay.minParam = 1e-5;
                shadowRay.maxParam = shadowDistance;
                if (world->isVisible(random, shadowRay, wavelength, transmission)) radiance += transmission * direct;
              }
            }
          }
        }

        // Sample next direction.
        throughput = 1;
        if (!isFiniteAndPositive(vertex.importanceSample(random, vertex.pathOmegaO, vertex.pathOmegaI, throughput))) break;
        throughput *= vertex.pathThroughput;
        if (!(throughput > 1e-6).any()) break;
      }

      // Update the ray.
      ray.org = vertex.position;
      ray.dir = vertex.pathOmegaI;
      ray.minParam = 1e-3;
      ray.maxParam = Inf;
      ray.derivatives = {}; // Ignore derivatives after the first bounce.

      // Add the vertex to the path.
      path.emplace_back(std::move(vertex));
    }
    if (problem.acceptPathContribution(path, radiance) == Status::Done) break;
  }
}

} // namespace d5b
