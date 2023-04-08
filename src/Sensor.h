#pragma once

#include "dirsig5b/Sensor.h"

class Sensor final : public d5b::Sensor {
public:

  Sensor() { image.resize(dim, dim, 3); }

  void request(std::vector<d5b::ProblemRequest> &problemRequests) override;

  [[nodiscard]] d5b::Status finish() override;

public:
  mi::Tensor<float, mi::TensorShape<mi::Dynamic, mi::Dynamic, mi::Dynamic>> image;

  d5b::Random seeds;

  size_t dim{2048};

  size_t count{0};
};
