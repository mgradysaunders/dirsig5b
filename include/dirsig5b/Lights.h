#pragma once

#include "dirsig5b/common.h"

namespace d5b {

struct DirectLight final {
public:
  /// Sample the solid angle of the light with respect to the given reference position and
  /// return the solid angle Probability Density Function (PDF) evaluation associated with
  /// the sample.
  ///
  /// \param[in]  random     The random generator for sampling.
  /// \param[in]  position   The reference position.
  /// \param[out] direction  The direction from the reference position to the light source.
  /// \param[out] distance   The distance from the reference position to the light source.
  /// \param[out] emission   The emission contribution.
  ///
  std::function<double(Random &random, Vector3 position, Vector3 &direction, double &distance, SpectralVector &emission)>
    importanceSampleSolidAngle;

  size_t numSubSamples{1};
};

} // namespace d5b
