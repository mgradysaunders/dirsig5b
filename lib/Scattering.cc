#include "dirsig5b/Scattering.h"

#include <Microcosm/Render/Phase>

namespace d5b {

void Scattering::initializeHenyeyGreenstein(double mu) {
  auto phase{mi::render::HenyeyGreensteinPhase(mu)};
  *this = Scattering{
    [=](Vector3 omegaO, Vector3 omegaI, SpectralVector &f) { f = phase.phase(omegaO, omegaI); },
    [=](Vector3 omegaO, Vector3 omegaI) -> double { return phase.phase(omegaO, omegaI); },
    [=](Random &random, Vector3 omegaO, Vector3 &omegaI, SpectralVector &) -> double {
      return phase.phase(omegaO, omegaI = phase.phaseSample(random));
    }};
}

void Scattering::initializeLambertian(double Lr, double Lt) {
  double Wr = Lr / (Lr + Lt);
  double Wt = Lt / (Lr + Lt);
  *this = Scattering{
    [=](Vector3 omegaO, Vector3 omegaI, SpectralVector &f) {
      if (signbit(omegaO[2]) == signbit(omegaI[2])) {
        f = Lr * OneOverPi * abs(omegaI[2]);
      } else {
        f = Lt * OneOverPi * abs(omegaI[2]);
      }
    },
    [=](Vector3 omegaO, Vector3 omegaI) -> double {
      if (signbit(omegaO[2]) == signbit(omegaI[2])) {
        return Wr * OneOverPi * abs(omegaI[2]);
      } else {
        return Wt * OneOverPi * abs(omegaI[2]);
      }
    },
    [=](Random &random, Vector3 omegaO, Vector3 &omegaI, SpectralVector &beta) -> double {
      omegaI = cosineHemisphereSample<double>(random);
      if (Wr == 1 || generateCanonical<double>(random) < Wr) {
        omegaI[2] = copysign(omegaI[2], +omegaO[2]), beta *= Wr;
        return Wr * OneOverPi * abs(omegaI[2]);
      } else {
        omegaI[2] = copysign(omegaI[2], -omegaO[2]), beta *= Wt;
        return Wt * OneOverPi * abs(omegaI[2]);
      }
    }};
}

void Scattering::initializeLinearCombination(std::vector<std::pair<double, Scattering>> weightAndTerm) {
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
      size_t i = 0;
      double u = generateCanonical<double>(random);
      for (const auto &[weight, term] : *sharedWeightAndTerm) {
        if (u <= weight || i + 1 == sharedWeightAndTerm->size()) {
          if (double p = term.importanceSample(random, omegaO, omegaI, beta); !(p > 0 && mi::isfinite(p))) {
            return 0;
          }
          break;
        } else {
          i += 1;
          u -= weight;
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

void Scattering::addTransform(const Transform &transform) {
  *this = Scattering{
    [transform, evaluateBSDF = std::move(evaluateBSDF)](Vector3 omegaO, Vector3 omegaI, SpectralVector &f) {
      omegaO = fastNormalize(transform.applyForward(Transform::Rule::Linear, omegaO));
      omegaI = fastNormalize(transform.applyForward(Transform::Rule::Linear, omegaI));
      evaluateBSDF(omegaO, omegaI, f);
    },
    [transform, evaluatePDF = std::move(evaluatePDF)](Vector3 omegaO, Vector3 omegaI) -> double {
      omegaO = fastNormalize(transform.applyForward(Transform::Rule::Linear, omegaO));
      omegaI = fastNormalize(transform.applyForward(Transform::Rule::Linear, omegaI));
      return evaluatePDF(omegaO, omegaI);
    },
    [transform, importanceSample = std::move(importanceSample)](
      Random &random, Vector3 omegaO, Vector3 &omegaI, SpectralVector &beta) -> double {
      omegaO = fastNormalize(transform.applyForward(Transform::Rule::Linear, omegaO));
      double p = importanceSample(random, omegaO, omegaI, beta);
      omegaI = fastNormalize(transform.applyInverse(Transform::Rule::Linear, omegaI));
      return p;
    }};
}

} // namespace d5b
