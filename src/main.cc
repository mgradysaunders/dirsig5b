#include "Sensor.h"
#include "World.h"
#include "dirsig5b/Simulation.h"

#include <iostream>

int main() {
  Sensor sensor;
  World world;
  world.initialize();
  d5b::Simulation simulation{};
  simulation.sensor = &sensor;
  simulation.world = &world;
  simulation.simulate();
  return 0;
}
