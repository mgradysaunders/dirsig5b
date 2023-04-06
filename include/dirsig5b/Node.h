#pragma once

#include "Ray.h"
#include "Scattering.h"

namespace d5b {

class Node final {
public:
  /// The position.
  Vector3 position;

  /// The outgoing direction (back to camera).
  Vector3 omegaO;

  /// The incident direction.
  Vector3 omegaI;

  /// The throughput at this node, cumulative product of BSDF over density.
  SpectralVector throughput;

  std::optional<SurfaceLocation> surfaceLocation;

  /// The scattering functions.
  std::optional<Scattering> scattering;

  /// The path length from the sensor up to this node.
  double pathLength{0};
};

} // namespace d5
