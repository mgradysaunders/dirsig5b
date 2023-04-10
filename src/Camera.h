#pragma once

#include "dirsig5b/Sensor.h"

#include <Microcosm/Timer>

class Camera final : public d5b::Sensor {
public:
  void initialize();

  void request(std::vector<d5b::ProblemRequest> &problemRequests) override;

  [[nodiscard]] d5b::Status finish() override;

public:
  std::string basename{"Render"};

  d5b::Transform localToWorld{};

  size_t seed{0};

  size_t sizeX{1920};

  size_t sizeY{1080};

  size_t wavelengthCount{20};

  double wavelengthMin{0.4};

  double wavelengthMax{0.7};

  double gain{2};

  double fovY{60.0_degrees};

  double dofRadius{0};

  double dofDistance{1000};

  size_t dofApertureBlades{5};

  size_t maxBounces{5};

  size_t maxSamples{256};

  size_t numSamplesPerBatch{4};

  struct Cache {
    d5b::Random seedSequence{};

    d5b::SpectralVector wavelength{};

    d5b::SpectralVector throughput{};

    double aspectRatio{1};

    double focalLength{0};

    size_t numSamples{0};
  } cache;

  mi::Tensor<float, mi::TensorShape<mi::Dynamic, mi::Dynamic, mi::Dynamic>> buffer;
};
