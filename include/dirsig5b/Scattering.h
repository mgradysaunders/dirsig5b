#pragma once

#include "dirsig5b/Transform.h"

namespace d5b {

class D5B_API Scattering final {
public:
  /// \name Scattering API
  ///
  /// Three functions form the primary hooks for the scattering API:
  /// 1. Evaluate the Bidirectional Scattering Distribution Function (BSDF).
  /// 2. Evaluate the Probability Density Function (PDF).
  /// 3. Importance sample the Probability Density Function (PDF).
  ///
  /// There are some understood conventions worth noting:
  /// - The _outgoing_ direction is denoted omegaO, and it points toward the sensor.
  /// - The _incident_ direction is denoted omegaI, and it points toward the light.
  /// - We assume directions are already normalized.
  /// - We assume directions are already transformed into the appropriate local coordinate system.
  ///
  /// These functions may either model scattering at surfaces, where the BSDF combines the
  /// Bidirectional Reflectance and Transmittance Distribution Functions (BRDF, BTDF), or inside
  /// volumes, where the BSDF instead goes by the term "phase function". At surfaces, the BSDF
  /// contains an implicit cosine term, specifically |cos(thetaI)|, to account for projection of
  /// incident light onto the surface. This is important to note because some rendering software
  /// explicitly factors this term out into the rest of the path tracing. However, at least for
  /// the scope of this project, it is much cleaner to allow it to be truly implicit.
  ///
  /// \{

  /// Evaluate the Bidirectional Scattering Distribution Function (BSDF).
  std::function<void(Vector3 omegaO, Vector3 omegaI, SpectralVector &f)> evaluateBSDF;

  /// Evaluate the Probability Density Function (PDF) for sampling omegaI given omegaO.
  std::function<double(Vector3 omegaO, Vector3 omegaI)> evaluatePDF;

  /// Importance sample omegaI conditionally given omegaO, and calculate the throughput ratio.
  ///
  /// This is understood to combine all relevant BSDF/PDF functionality simultaneously, because there
  /// is usually significant cancellation of terms that the implementation can perform to prevent floating
  /// point blow-ups and either speed up calculation (fewer operations) or convergence (fewer samples needed
  /// in the end).
  ///
  /// 1. Sample omegaI conditionally given omegaO according to the PDF.
  /// 2. Evaluate the BSDF for the pair of omegaO and omegaI.
  /// 3. Evaluate the PDF for the pair of omegaO and omegaI.
  /// 4. Evaluate the "throughput", here called beta, which is the BSDF evaluation divided by the
  ///    PDF evaluation. This is where there tends to be significant term cancellation, because the
  ///    ideal importance sampling routine will have the PDF exactly proportional to the BSDF.
  ///
  std::function<double(Random &random, Vector3 omegaO, Vector3 &omegaI, SpectralVector &beta)> importanceSample;

  /// \}

public:
  /// Initialize as Henyey-Greenstein phase BSDF with the given mean scattering cosine.
  void setHenyeyGreenstein(double meanCosine);

  /// Initialize as Lambertian BSDF with the given reflectance and transmittance.
  void setLambertian(double fractionR, double fractionT);

  /// Initialize as linear mixture of other scattering functions.
  void setLinearMixture(std::vector<std::pair<double, Scattering>> weightAndTerm);

  /// Multiply with spectral fraction.
  void multiply(SpectralVector fraction);
};

using ScatteringProvider = std::function<void(const SpectralVector &wavelength, Scattering &scattering)>;

} // namespace d5b
