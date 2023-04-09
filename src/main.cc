#include "LumberyardBistro.h"
#include "PerspectiveCamera.h"
#include "dirsig5b/Simulation.h"

#include <iostream>

int main() {
  PerspectiveCamera perspectiveCamera;
  perspectiveCamera.localToWorld = d5b::DualQuaternion::lookAt({0, 400, 1000}, {0, 400, 0}, {0, 1, 0});
  perspectiveCamera.sizeX = 1920 * 2;
  perspectiveCamera.sizeY = 1080 * 2;
  perspectiveCamera.fovY = 75.0_degrees;
  perspectiveCamera.dofRadius = 1;
  perspectiveCamera.dofDistance = 600;
  perspectiveCamera.maxBounces = 5;
  perspectiveCamera.maxSamples = 1024;
  perspectiveCamera.initialize();
  LumberyardBistro lumberyardBistro;
  lumberyardBistro.initialize();
  d5b::Simulation simulation{};
  simulation.sensor = &perspectiveCamera;
  simulation.world = &lumberyardBistro;
  simulation.simulate();
  return 0;
}
