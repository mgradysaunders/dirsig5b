#pragma once

#include "dirsig5b/LocalVolume.h"
#include "dirsig5b/Ray.h"

namespace d5b {

class Medium {
public:
  virtual ~Medium() = default;

  [[nodiscard]] virtual bool intersect(Random &random, Ray ray, LocalVolume &localVolume) const = 0;
};

} // namespace d5b
