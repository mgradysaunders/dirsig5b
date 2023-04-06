#pragma once

#include "dirsig5b/LocalSurface.h"
#include "dirsig5b/Ray.h"

namespace d5b {

class Medium;

class D5B_API World {
public:
  virtual ~World() = default;

  [[nodiscard]] virtual const Medium *mediumLookup(Vector3) const { return nullptr; }

  [[nodiscard]] virtual bool intersect(Random &random, Ray ray, LocalSurface &localSurface) const = 0;
};

} // namespace d5b
