#pragma once

#include <Microcosm/Pcg>
#include <Microcosm/Quaternion>
#include <Microcosm/shape>
#include <Microcosm/tensor>
#include <functional>
#include <memory>

#include "dirsig5b/Export.h"

namespace d5b {

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
using SpectralVector = mi::Vectord;

using mi::abs;
using mi::copysign;
using mi::cosineHemisphereSample;
using mi::distance;
using mi::distanceSquare;
using mi::dot;
using mi::fastLength;
using mi::fastNormalize;
using mi::generateCanonical;
using mi::inverse;
using mi::length;
using mi::lengthSquare;
using mi::normalize;
using mi::signbit;
using mi::Slice;

enum class Status { NotDone, Done };

} // namespace d5b
