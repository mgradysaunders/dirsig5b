#pragma once

#include "dirsig5b/Sensor.h"

class Sensor final : public d5b::Sensor {
public:
  [[nodiscard]] d5b::Status recordProblemRequests(std::vector<d5b::ProblemRequest> &problemRequests) override;

  float image[512][512][3]{};
};
