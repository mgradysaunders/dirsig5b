#include "Sensor.h"

#include <Microcosm/stbi>
#include <iostream>

void Sensor::request(std::vector<d5b::ProblemRequest> &problemRequests) {
  problemRequests.reserve(dim * dim);
  for (int y = 0; y < dim; y++) {
    for (int x = 0; x < dim; x++) {
      problemRequests.emplace_back([=, this] {
        d5b::Problem problem;
        problem.wavelength = {0.6, 0.5, 0.4, 0.3};
        problem.throughput = {1.0, 1.0, 1.0, 1.0};
        problem.seed = seeds();
        problem.sampleRay = [=, this](d5b::Random &random) {
          double dx = (x + d5b::generateCanonical<double>(random)) / dim - 0.5;
          double dy = (y + d5b::generateCanonical<double>(random)) / dim - 0.5;
          return d5b::Ray{d5b::Vector3(0, 500, 1000), d5b::normalize(d5b::Vector3(2 * dx, -2 * dy, -1))};
        };
        problem.acceptPathContribution = [=, this, sampleIndex = std::make_shared<size_t>(0)](
                                           const std::vector<d5b::Vertex> &path, const d5b::SpectralVector &radiance) {
          image(y, x, 0) += radiance[0];
          image(y, x, 1) += radiance[1];
          image(y, x, 2) += radiance[2];
          return (*sampleIndex)++ >= 4 ? d5b::Status::Done : d5b::Status::NotDone;
        };
        return problem;
      });
    }
  }
}

d5b::Status Sensor::finish() {
  std::cout << "Finished 4 more samples!" << std::endl;
  count += 4;
  mi::stbi::ImageU8 imageU8(mi::with_shape, dim, dim, 3);
  for (size_t y = 0; y < dim; y++) {
    for (size_t x = 0; x < dim; x++) {
      imageU8(y, x, mi::Slice()).assign(255 * mi::encodeSRGB(mi::tonemapACES(mi::Vector3d(image(y, x, mi::Slice()) / count))));
    }
  }
  mi::stbi::saveU8(mi::stbi::FileType::PNG, "Test.png", imageU8);
  return count >= 128 ? d5b::Status::Done : d5b::Status::NotDone;
}
