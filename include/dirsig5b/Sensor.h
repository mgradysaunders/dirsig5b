#pragma once

#include "dirsig5b/Ray.h"
#include "dirsig5b/Vertex.h"

namespace d5b {

class D5B_API Problem final {
public:
  size_t seed{0};

  size_t bounceLimit{0};

  SpectralVector wavelength;

  SpectralVector throughput;

  std::function<Ray(Random &random)> sampleRay;

  std::function<Status(const std::vector<Vertex> &path, const SpectralVector &radiance)> acceptPathContribution;
};

using ProblemRequest = std::function<Problem()>;

class D5B_API Sensor {
public:
  virtual ~Sensor() = default;

  virtual void request(std::vector<ProblemRequest> &problemRequests) = 0;

  [[nodiscard]] virtual Status finish() { return Status::Done; }
};

} // namespace d5b
