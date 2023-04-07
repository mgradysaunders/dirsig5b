#include "Sensor.h"

d5b::Status Sensor::recordProblemRequests(std::vector<d5b::ProblemRequest> &problemRequests) {
  problemRequests.reserve(512 * 512);
  for (int y = 0; y < 512; y++) {
    for (int x = 0; x < 512; x++) {
      problemRequests.emplace_back([=, this] {
        d5b::Problem problem;
        problem.wavelength = {0.4, 0.5, 0.6, 0.7};
        problem.throughput = {1.0, 1.0, 1.0, 1.0};
        problem.seed = x * 512 + y;
        problem.sampleRay = [=, this](d5b::Random &random) {
          double dx = (x + d5b::generateCanonical<double>(random)) / 512.0 - 0.5;
          double dy = (y + d5b::generateCanonical<double>(random)) / 512.0 - 0.5;
          return d5b::Ray{d5b::Vector3(0, 0, 4), d5b::normalize(d5b::Vector3(dx, -dy, -1))};
        };
        problem.acceptPathContribution = [=, this, sampleIndex = std::make_shared<size_t>(0)](
                                           const std::vector<d5b::Vertex> &path, const d5b::SpectralVector &radiance) {
          if (!path.empty()) {
            image[y][x][0] += radiance[0];
            image[y][x][1] += radiance[1];
            image[y][x][2] += radiance[2];
          }
          return (*sampleIndex)++ > 16 ? d5b::Status::Done : d5b::Status::NotDone;
        };
        return problem;
      });
    }
  }
  return d5b::Status::Done;
}
