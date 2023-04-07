#include "dirsig5b/Simulation.h"
#include "Sensor.h"
#include "World.h"

#include <iostream>

int main() {
  auto mesh = mi::geometry::Mesh(mi::geometry::FileOBJ("Suzanne.obj"));
  mesh.subdivide();
  mesh.calculateNormals();
  Sensor sensor;
  World world;
  world.triangleMesh.buildFrom(mesh);
  d5b::Simulation simulation{};
  simulation.sensor = &sensor;
  simulation.world = &world;
  simulation.simulate();
  std::cout << "P3\n";
  std::cout << "512 512\n";
  std::cout << "255\n";
  for (size_t y = 0; y < 512; y++) {
    for (size_t x = 0; x < 512; x++) {
      std::cout << int(255 * mi::encodeSRGB((sensor.image[y][x][0]) / 16)) << ' ';
      std::cout << int(255 * mi::encodeSRGB((sensor.image[y][x][1]) / 16)) << ' ';
      std::cout << int(255 * mi::encodeSRGB((sensor.image[y][x][2]) / 16)) << '\n';
    }
    std::cout << std::endl;
  }
  return 0;
}
