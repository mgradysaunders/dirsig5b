#pragma once

#include "dirsig5b/Transform.h"

namespace d5b {

class Medium;

class D5B_API Ray final {
public:
  /// Calculate point at the given parameter.
  [[nodiscard]] Vector3 operator()(double param) const noexcept { return org + param * dir; }

  /// Calculate parameter of the given point.
  [[nodiscard]] double parameterOf(Vector3 point) const noexcept { return dot(point - org, dir) / lengthSquare(dir); }

  /// Is parameter in allowed range?
  [[nodiscard]] bool isParameterInRange(double param) const noexcept { return minParam <= param && param < maxParam; }

  /// Is infinite in one or both directions?
  [[nodiscard]] bool isInfinite() const noexcept { return mi::isinf(minParam, maxParam); }

  /// Apply coordinate transformation.
  Ray &withTransform(const Transform &transform) noexcept {
    org = transform.applyForward(Transform::Rule::Affine, org);
    dir = transform.applyForward(Transform::Rule::Linear, dir);
    return *this;
  }

public:
  /// The ray origin.
  Vector3 org;

  /// The ray direction.
  Vector3 dir;

  /// The minimum parameter.
  double minParam{0};

  /// The maximum parameter.
  double maxParam{Inf};

  /// The time.
  double time{0};

  /// The current medium.
  const Medium *medium{nullptr};
};

} // namespace d5b
