#pragma once

#include "dirsig5b/Ray.h"

namespace d5b {

class SensorRay final {
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

  std::function<SpectralVector()> initialThroughput;

  std::function<void(const SpectralVector &)> processRadiance;

  size_t maxBounces{0};
};

class Sensor {
public:
  virtual ~Sensor() = default;

  enum class Status { MoreTodo, Done };

  [[nodiscard]] virtual Status populateSensorRays(std::vector<SensorRay> &sensorRays) const = 0;
};

} // namespace d5b
