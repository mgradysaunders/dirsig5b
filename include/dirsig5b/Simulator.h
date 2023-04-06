#pragma once

#include "dirsig5b/Sensor.h"
#include "dirsig5b/World.h"

namespace d5b {

class Simulator {
public:
  const Sensor *sensor{nullptr};

  const World *world{nullptr};

  void simulate();

  void simulate(const SensorRay &sensorRay);
};

} // namespace d5b
