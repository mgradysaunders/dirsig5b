#include "dirsig5b/Scattering.h"

#include <Microcosm/Render/DiffuseModels>
#include <Microcosm/Render/Phase>

namespace d5b {

void Scattering::setHenyeyGreenstein(double meanCosine) {
  auto phase{mi::render::HenyeyGreensteinPhase(meanCosine)};
  *this = Scattering{
    [=](Vector3 omegaO, Vector3 omegaI, SpectralVector &f) { f = phase.phase(omegaO, omegaI); },
    [=](Vector3 omegaO, Vector3 omegaI) -> double { return phase.phase(omegaO, omegaI); },
    [=](Random &random, Vector3 omegaO, Vector3 &omegaI, SpectralVector &) -> double {
      return phase.phase(omegaO, omegaI = phase.phaseSample(random));
    }};
}

void Scattering::setLambertDiffuse(double fractionR, double fractionT) {
  double weightR = fractionR / (fractionR + fractionT);
  double weightT = fractionT / (fractionR + fractionT);
  *this = Scattering{
    [=](Vector3 omegaO, Vector3 omegaI, SpectralVector &f) {
      if (signbit(omegaO[2]) == signbit(omegaI[2])) {
        f = fractionR * OneOverPi * abs(omegaI[2]);
      } else {
        f = fractionT * OneOverPi * abs(omegaI[2]);
      }
    },
    [=](Vector3 omegaO, Vector3 omegaI) -> double {
      if (signbit(omegaO[2]) == signbit(omegaI[2])) {
        return weightR * OneOverPi * abs(omegaI[2]);
      } else {
        return weightT * OneOverPi * abs(omegaI[2]);
      }
    },
    [=](Random &random, Vector3 omegaO, Vector3 &omegaI, SpectralVector &beta) -> double {
      omegaI = cosineHemisphereSample<double>(random);
      if (weightR == 1 || generateCanonical<double>(random) < weightR) {
        omegaI[2] = copysign(omegaI[2], +omegaO[2]), beta *= weightR;
        return weightR * OneOverPi * abs(omegaI[2]);
      } else {
        omegaI[2] = copysign(omegaI[2], -omegaO[2]), beta *= weightT;
        return weightT * OneOverPi * abs(omegaI[2]);
      }
    }};
}

void Scattering::setDisneyDiffuse(double roughness, double lambert, double retro, double sheen) {
  setLambertDiffuse(1, 0);
  evaluateBSDF = [=](Vector3 omegaO, Vector3 omegaI, SpectralVector &f) {
    if (signbit(omegaO[2]) == signbit(omegaI[2])) {
      f = mi::render::DisneyDiffuseBRDF{roughness, lambert, retro, sheen}.scatter(omegaO, omegaI);
    } else {
      f = 0;
    }
  };
}

void Scattering::setOrenNayarDiffuse(double sigma) {
  setLambertDiffuse(1, 0);
  evaluateBSDF = [=](Vector3 omegaO, Vector3 omegaI, SpectralVector &f) {
    if (signbit(omegaO[2]) == signbit(omegaI[2])) {
      f = mi::render::OrenNayarBRDF{sigma}.scatter(omegaO, omegaI);
    } else {
      f = 0;
    }
  };
}

void Scattering::setLinearMixture(std::vector<std::pair<double, Scattering>> weightAndTerm) {
  double factor{0};
  for (auto &[weight, term] : weightAndTerm) weight = mi::max(weight, 0), factor += weight;
  for (auto &[weight, term] : weightAndTerm) weight /= factor;
  auto sharedWeightAndTerm = std::make_shared<std::vector<std::pair<double, Scattering>>>(std::move(weightAndTerm));
  *this = Scattering{
    [=](Vector3 omegaO, Vector3 omegaI, SpectralVector &f) {
      f = 0;
      SpectralVector g(f.shape);
      for (const auto &[weight, term] : *sharedWeightAndTerm)
        if (weight > 0) term.evaluateBSDF(omegaO, omegaI, g), f += weight * g;
      f *= factor;
    },
    [=](Vector3 omegaO, Vector3 omegaI) -> double {
      double p = 0;
      for (const auto &[weight, term] : *sharedWeightAndTerm)
        if (weight > 0) p += weight * term.evaluatePDF(omegaO, omegaI);
      return p;
    },
    [=](Random &random, Vector3 omegaO, Vector3 &omegaI, SpectralVector &beta) -> double {
      SpectralVector &f = beta;
      SpectralVector g(f.shape);
      double u = generateCanonical<double>(random);
      size_t i = 0;
      for (const auto &[weight, term] : *sharedWeightAndTerm) {
        if (u <= weight || i + 1 == sharedWeightAndTerm->size()) {
          if (!isFiniteAndPositive(term.importanceSample(random, omegaO, omegaI, beta))) {
            return 0;
          }
          break;
        } else {
          u -= weight;
          i += 1;
        }
      }
      f = 0;
      double p = 0;
      for (const auto &[weight, term] : *sharedWeightAndTerm)
        if (weight > 0) {
          term.evaluateBSDF(omegaO, omegaI, g);
          f += weight * g;
          p += weight * term.evaluatePDF(omegaO, omegaI);
        }
      f *= factor / p;
      if (!mi::isfinite(f).all()) return 0;
      return p;
    }};
}

void Scattering::multiply(SpectralVector fraction) {
  auto sharedFraction{std::make_shared<SpectralVector>(std::move(fraction))};
  *this = Scattering{
    [sharedFraction, evaluateBSDF = std::move(evaluateBSDF)](Vector3 omegaO, Vector3 omegaI, SpectralVector &f) {
      evaluateBSDF(omegaO, omegaI, f), f *= (*sharedFraction);
    },
    std::move(evaluatePDF),
    [sharedFraction, importanceSample = std::move(importanceSample)](
      Random &random, Vector3 omegaO, Vector3 &omegaI, SpectralVector &beta) -> double {
      double p = importanceSample(random, omegaO, omegaI, beta);
      beta *= (*sharedFraction);
      return p;
    }};
}

} // namespace d5b
