#pragma once

#include "dirsig5b/Scattering.h"

namespace d5b {

class Medium;

class D5B_API LocalSurface final {
public:
  /// Apply coordinate transformation.
  LocalSurface &withTransform(const Transform &transform) noexcept;

public:
  /// The position.
  Vector3 position;

  /// The texture coordinate.
  Vector2 texcoord;

  /// The tangent vectors.
  Vector3 tangents[2]{};

  /// The normal vector.
  Vector3 normal;

  /// The local-to-world orthonormal basis.
  [[nodiscard]] Matrix3 localToWorld() const noexcept {
    Vector3 tangent = normalize(tangents[0] - dot(tangents[0], normal) * normal);
    if ((tangent == 0).all() || !mi::isfinite(tangent).all()) return Matrix3::orthonormalBasis(normal);
    Matrix3 basis;
    basis.col(0).assign(fastNormalize(tangent));
    basis.col(1).assign(fastNormalize(cross(normal, tangent)));
    basis.col(2).assign(normal);
    return basis;
  }

  /// The medium above the surface.
  const Medium *mediumAbove{nullptr};

  /// The medium below the surface.
  const Medium *mediumBelow{nullptr};

  /// The scattering provider.
  ScatteringProvider scatteringProvider{};
};

} // namespace d5b
