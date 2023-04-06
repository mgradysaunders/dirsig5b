#include "PathTracer.h"

namespace d5 {

std::vector<Node> PathTracer::generatePath(Random &random, Ray ray, SpectralVector throughput) const {
  std::vector<Node> nodes;
  while (true) {
    Intersection intersection;
    if (!world->intersect(ray, &intersection)) break;

    Node node;
    node.position = intersection.position;
    node.intersection = intersection;
    node.scattering = intersection.getScattering(wavelengths);
    node.throughput = throughput;
    node.omegaO = mi::fastNormalize(-ray.dir);
    if (double p = node.scattering->importanceSample(random, node.omegaO, node.omegaI, throughput);
        !(p > 0 && mi::isfinite(p))) {
      break;
    }
    ray.org = node.position;
    ray.dir = node.omegaI;
    throughput *= node.throughput;
    nodes.emplace_back(std::move(node));
  }
  return nodes;
}

} // namespace d5
