#pragma once

#include "dirsig5b/Ray.h"
#include "dirsig5b/Vertex.h"

namespace d5b {

class D5B_API SensorRay final {
public:
  Ray ray;

  struct Derivatives {
    /// The derivative of the origin with respect to image-plane X.
    Vector3 orgDerivWrtX;

    /// The derivative of the origin with respect to image-plane Y.
    Vector3 orgDerivWrtY;

    /// The derivative of the direction with respect to image-plane X.
    Vector3 dirDerivWrtX;

    /// The derivative of the direction with respect to image-plane Y.
    Vector3 dirDerivWrtY;
  };

  std::optional<Derivatives> derivatives;
};

class D5B_API Problem final {
public:
  SpectralVector wavelength;

  SpectralVector throughput;

  size_t seed{0};

  size_t bounceLimit{0};

  std::function<SensorRay(Random &random)> sampleSensorRay;

  std::function<Status(size_t numSamples, const std::vector<Vertex> &path, const SpectralVector &radiance)> acceptContribution;
};

using ProblemRequest = std::function<Problem()>;

class D5B_API Sensor {
public:
  virtual ~Sensor() = default;

  [[nodiscard]] virtual Status request(std::vector<ProblemRequest> &problemRequests) = 0;
};

} // namespace d5b
