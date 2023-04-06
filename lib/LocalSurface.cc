#include "dirsig5b/LocalSurface.h"

namespace d5b {

LocalSurface &LocalSurface::withTransform(const Transform &transform) noexcept {
  position = transform.applyForward(Transform::Rule::Affine, position);
  tangents[0] = transform.applyForward(Transform::Rule::Linear, tangents[0]);
  tangents[1] = transform.applyForward(Transform::Rule::Linear, tangents[1]);
  normal = transform.applyForward(Transform::Rule::Normal, normal);
  return *this;
}

} // namespace d5b
