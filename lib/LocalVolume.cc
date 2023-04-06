#include "dirsig5b/LocalVolume.h"

namespace d5b {

LocalVolume &LocalVolume::withTransform(const Transform &transform) noexcept {
  position = transform.applyForward(Transform::Rule::Affine, position);
  localToWorld.col(0).assign(transform.applyForward(Transform::Rule::Linear, localToWorld.col(0)));
  localToWorld.col(1).assign(transform.applyForward(Transform::Rule::Linear, localToWorld.col(1)));
  localToWorld.col(2).assign(transform.applyForward(Transform::Rule::Linear, localToWorld.col(2)));
  return *this;
}

} // namespace d5b
