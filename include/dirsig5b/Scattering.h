#pragma once

#include "dirsig5b/Transform.h"

namespace d5b {

class D5B_API Scattering final {
public:
  void initializeHenyeyGreenstein(double mu);

  void initializeLambertian(double Lr, double Lt);

  void initializeLinearCombination(std::vector<std::pair<double, Scattering>> weightAndTerm);

  void addTransform(const Transform &transform);

public:
  std::function<void(Vector3 omegaO, Vector3 omegaI, SpectralVector &f)> evaluateBSDF;

  std::function<double(Vector3 omegaO, Vector3 omegaI)> evaluatePDF;

  std::function<double(Random &random, Vector3 omegaO, Vector3 &omegaI, SpectralVector &beta)> importanceSample;
};

} // namespace d5b
