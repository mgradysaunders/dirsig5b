#pragma once

#include "dirsig5b/Transform.h"

namespace d5b {

class D5B_API Scattering final {
public:
  void initializeHenyeyGreenstein(double meanCosine);

  void initializeLambertian(double fractionR, double fractionT);

  void initializeLinearCombination(std::vector<std::pair<double, Scattering>> weightAndTerm);

public:
  std::function<void(Vector3 omegaO, Vector3 omegaI, SpectralVector &f)> evaluateBSDF;

  std::function<double(Vector3 omegaO, Vector3 omegaI)> evaluatePDF;

  std::function<double(Random &random, Vector3 omegaO, Vector3 &omegaI, SpectralVector &beta)> importanceSample;
};

using ScatteringProvider = std::function<void(const SpectralVector &wavelength, Scattering &scattering)>;

} // namespace d5b
