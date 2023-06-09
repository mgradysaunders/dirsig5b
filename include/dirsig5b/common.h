#pragma once

#include <Microcosm/Pcg>
#include <Microcosm/Quaternion>
#include <Microcosm/Render/Scattering>
#include <Microcosm/Render/Spectrum>
#include <Microcosm/Render/common>
#include <Microcosm/Tensor>
#include <functional>
#include <memory>

#include "dirsig5b/Export.h"

namespace d5b {

enum class Status { NotDone, Done };

constexpr double Pi = mi::constants::Pi<double>;
constexpr double OneOverPi = mi::constants::OneOverPi<double>;
constexpr double LightSpeed = mi::constants::LightSpeed<double>;
constexpr double Inf = mi::constants::Inf<double>;
constexpr double NaN = mi::constants::NaN<double>;

using Random = mi::Pcg32;
using Vector2 = mi::Vector2d;
using Vector3 = mi::Vector3d;
using Matrix3 = mi::Matrix3d;
using Matrix4 = mi::Matrix4d;
using Quaternion = mi::Quaterniond;
using DualQuaternion = mi::DualQuaterniond;
using SpectralVector = mi::render::Spectrum;

using mi::abs;
using mi::copysign;
using mi::distance;
using mi::distanceSquare;
using mi::dot;
using mi::fastLength;
using mi::fastNormalize;
using mi::inverse;
using mi::length;
using mi::lengthSquare;
using mi::normalize;
using mi::randomize;
using mi::signbit;
using mi::Slice;
using mi::render::cosineHemisphereSample;

[[nodiscard]] inline bool isFiniteAndPositive(double x) noexcept { return x > 0 && mi::isfinite(x); }
[[nodiscard]] inline bool isFiniteAndPositive(const SpectralVector &v) noexcept {
  return (v > 0).any() && mi::isfinite(v).all();
}

using Scattering = mi::render::Scattering;
using ScatteringProvider = std::function<void(const SpectralVector &wavelength, Scattering &scattering)>;

} // namespace d5b
