#pragma once

#include "dirsig5b/Ray.h"

namespace d5b {

class Scattering;

class Medium;

class D5B_API SurfaceLocation final {
public:
  /// Apply coordinate transformation.
  SurfaceLocation &withTransform(const Transform &transform) noexcept {
    position = transform.applyForward(Transform::Rule::Affine, position);
    tangents[0] = transform.applyForward(Transform::Rule::Linear, tangents[0]);
    tangents[1] = transform.applyForward(Transform::Rule::Linear, tangents[1]);
    normal = transform.applyForward(Transform::Rule::Normal, normal);
    return *this;
  }

public:
  /// The position.
  Vector3 position;

  /// The texture coordinate.
  Vector2 texcoord;

  /// The tangent vectors.
  Vector3 tangents[2]{};

  /// The normal vector.
  Vector3 normal;

  /// The medium above the surface.
  const Medium *mediumAbove{nullptr};

  /// The medium below the surface.
  const Medium *mediumBelow{nullptr};

  /// The scattering provider.
  std::function<std::unique_ptr<Scattering>(const SpectralVector &wavelengths)> getScattering;
};

} // namespace d5b
