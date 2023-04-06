#pragma once

#include "dirsig5b/Ray.h"
#include "dirsig5b/SurfaceLocation.h"

namespace d5b {

class World {
public:
  virtual ~World() = default;

  [[nodiscard]] virtual bool intersect(const Ray &ray, SurfaceLocation &location) const = 0;
};

} // namespace d5b
