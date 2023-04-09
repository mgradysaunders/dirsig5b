#include "LumberyardBistro.h"
#include "PerspectiveCamera.h"
#include "dirsig5b/Simulation.h"

#include <iostream>

int main() {
  PerspectiveCamera perspectiveCamera;
  perspectiveCamera.localToWorld = d5b::DualQuaternion::translate({0, 400, 1000});
  perspectiveCamera.initialize();
  LumberyardBistro lumberyardBistro;
  lumberyardBistro.initialize();
  d5b::Simulation simulation{};
  simulation.sensor = &perspectiveCamera;
  simulation.world = &lumberyardBistro;
  simulation.simulate();
  return 0;
}
