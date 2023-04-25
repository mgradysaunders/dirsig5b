#pragma once

#include "dirsig5b/Transform.h"

namespace d5b {

class Medium;

class D5B_API LocalVolume final {
public:
  /// Apply coordinate transformation.
  LocalVolume &withTransform(const Transform &transform) noexcept;

public:
  /// The position.
  Vector3 position;

  /// The local-to-world orthonormal basis.
  Matrix3 localToWorld{Matrix3::identity()};

  /// The medium.
  const Medium *medium{nullptr};

  /// The scattering provider.
  ScatteringProvider scatteringProvider{};
};

} // namespace d5b
