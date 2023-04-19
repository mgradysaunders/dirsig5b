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

  [[nodiscard]] operator mi::Ray3f() const noexcept { return {org, dir, float(minParam), float(maxParam)}; }

  /// Apply coordinate transformation.
  Ray &withTransform(const Transform &transform) noexcept {
    org = transform.applyForward(Transform::Rule::Affine, org);
    dir = transform.applyForward(Transform::Rule::Linear, dir);
    if (derivatives) {
      derivatives->orgDerivWrtX = transform.applyForward(Transform::Rule::Linear, derivatives->orgDerivWrtX);
      derivatives->orgDerivWrtY = transform.applyForward(Transform::Rule::Linear, derivatives->orgDerivWrtY);
      derivatives->dirDerivWrtX = transform.applyForward(Transform::Rule::Linear, derivatives->dirDerivWrtX);
      derivatives->dirDerivWrtY = transform.applyForward(Transform::Rule::Linear, derivatives->dirDerivWrtY);
    }
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

  struct Derivatives {
    /// The derivative of the origin with respect to X.
    Vector3 orgDerivWrtX;

    /// The derivative of the origin with respect to Y.
    Vector3 orgDerivWrtY;

    /// The derivative of the direction with respect to X.
    Vector3 dirDerivWrtX;

    /// The derivative of the direction with respect to Y.
    Vector3 dirDerivWrtY;
  };

  /// The derivatives with respect to the image plane if applicable.
  std::optional<Derivatives> derivatives{};
};

} // namespace d5b
