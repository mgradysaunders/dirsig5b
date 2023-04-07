#include "Sensor.h"
#include "World.h"
#include "dirsig5b/Simulation.h"

#include <iostream>

int main() {
  Sensor sensor;
  World world;
  {
    auto fileOBJ = mi::geometry::FileOBJ("/home/michael/Documents/Scenes/lumberyard-bistro/exterior.obj");
    auto fileMTL = mi::geometry::FileOBJ("/home/michael/Documents/Scenes/lumberyard-bistro/exterior.mtl");
    auto mesh = mi::geometry::Mesh(fileOBJ);
    world.triangleMesh.buildFrom(mesh, 2);
  }
  d5b::Simulation simulation{};
  simulation.sensor = &sensor;
  simulation.world = &world;
  simulation.simulate();
  return 0;
}
