#pragma once

#include "dirsig5b/common.h"

namespace d5b {

class Transform final {
public:
  /// The transform _rule_ or sense of the target object we want to transform.
  enum class Rule {
    Affine, ///< Affine transform, as if to points or positions.
    Linear, ///< Linear transform, as if to vectors.
    Normal  ///< Normal transform, so inverse transpose of linear transform.
  };

  Transform() = default;

  Transform(std::in_place_t, const Matrix4 &matrixFwd, const Matrix4 &matrixInv) noexcept : mFwd(matrixFwd), mInv(matrixInv) {}

  Transform(const Matrix4 &matrix) : mFwd(matrix), mInv(inverse(matrix)) {}

  Transform(const Matrix3 &matrix) {
    mFwd(Slice<0, 3>(), Slice<0, 3>()).assign(matrix);
    mInv(Slice<0, 3>(), Slice<0, 3>()).assign(inverse(matrix));
  }

  Transform(const Quaternion &quaternion) noexcept {
    mFwd(Slice<0, 3>(), Slice<0, 3>()).assign(Matrix3(quaternion));
    mInv(Slice<0, 3>(), Slice<0, 3>()).assign(mi::transpose(Matrix3(quaternion)));
  }

  Transform(const DualQuaternion &quaternion) : Transform(Matrix4(quaternion)) {}

  [[nodiscard]] Vector3 applyForward(Rule rule, Vector3 coord) const noexcept {
    switch (rule) {
    default:
    case Rule::Affine: return mi::dot(mFwd(Slice<0, 3>(), Slice<0, 3>()), coord) + mFwd(Slice<0, 3>(), 3);
    case Rule::Linear: return mi::dot(mFwd(Slice<0, 3>(), Slice<0, 3>()), coord);
    case Rule::Normal: return mi::dot(coord, mInv(Slice<0, 3>(), Slice<0, 3>()));
    }
    // Unreachable.
    return {};
  }

  [[nodiscard]] Vector3 applyInverse(Rule rule, Vector3 coord) const noexcept {
    switch (rule) {
    default:
    case Rule::Affine: return mi::dot(mInv(Slice<0, 3>(), Slice<0, 3>()), coord) + mInv(Slice<0, 3>(), 3);
    case Rule::Linear: return mi::dot(mInv(Slice<0, 3>(), Slice<0, 3>()), coord);
    case Rule::Normal: return mi::dot(coord, mFwd(Slice<0, 3>(), Slice<0, 3>()));
    }
    // Unreachable.
    return {};
  }

  [[nodiscard]] Vector3 basisX() const noexcept { return mFwd(Slice<0, 3>(), 0); }

  [[nodiscard]] Vector3 basisY() const noexcept { return mFwd(Slice<0, 3>(), 1); }

  [[nodiscard]] Vector3 basisZ() const noexcept { return mFwd(Slice<0, 3>(), 2); }

  [[nodiscard]] Vector3 origin() const noexcept { return mFwd(Slice<0, 3>(), 3); }

  [[nodiscard]] Transform inverted() const noexcept { return Transform(std::in_place, mInv, mFwd); }

  [[nodiscard]] Transform operator*(const Transform &other) const noexcept {
    return Transform(std::in_place, mi::dot(mFwd, other.mFwd), mi::dot(other.mInv, mInv));
  }

private:
  Matrix4 mFwd{Matrix4::identity()};

  Matrix4 mInv{Matrix4::identity()};
};

class MotionTransform final {
public:
  using Function = std::function<Transform(double)>;

  MotionTransform() = default;

  MotionTransform(const Transform &transform) : mFunc([=](double) -> Transform { return transform; }) {}

  MotionTransform(Function function) : mFunc(std::move(function)) {}

  [[nodiscard]] Transform operator()(double time) const { return mFunc(time); }

  [[nodiscard]] MotionTransform operator*(MotionTransform other) const {
    return MotionTransform(
      [motionA = *this, motionB = std::move(other)](double time) { return motionA(time) * motionB(time); });
  }

  [[nodiscard]] MotionTransform inverted() const {
    return MotionTransform([motion = *this](double time) { return motion(time).inverted(); });
  }

private:
  Function mFunc = [](double) -> Transform { return Transform(); };
};

} // namespace d5b
