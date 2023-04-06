#pragma once

#include "dirsig5b/Medium.h"
#include "dirsig5b/Sensor.h"
#include "dirsig5b/Vertex.h"
#include "dirsig5b/World.h"

namespace d5b {

class D5B_API Simulation {
public:
  Sensor *sensor{nullptr};

  const World *world{nullptr};

  void simulate();

  void simulate(Problem problem);
};

} // namespace d5b
