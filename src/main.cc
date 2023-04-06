#include "dirsig5b/Simulation.h"

#include <Microcosm/Geometry/FileOBJ>
#include <Microcosm/Geometry/Mesh>
#include <Microcosm/Render/TriangleMesh>

#include <iostream>

class MySensor final : public d5b::Sensor {
public:
  [[nodiscard]] d5b::Status recordProblemRequests(std::vector<d5b::ProblemRequest> &problemRequests) override {
    problemRequests.reserve(512 * 512);
    for (int y = 0; y < 512; y++) {
      for (int x = 0; x < 512; x++) {
        problemRequests.emplace_back([=, this] {
          d5b::Problem problem;
          problem.wavelength = {0.4, 0.5, 0.6, 0.7};
          problem.throughput = {1.0, 1.0, 1.0, 1.0};
          problem.seed = x * 512 + y;
          problem.sampleRay = [=, this](d5b::Random &random) {
            double dx = (x + d5b::generateCanonical<double>(random)) / 512.0 - 0.5;
            double dy = (y + d5b::generateCanonical<double>(random)) / 512.0 - 0.5;
            return d5b::Ray{d5b::Vector3(0, 0, 4), d5b::normalize(d5b::Vector3(dx, -dy, -1))};
          };
          problem.acceptPathContribution = [=, this, sampleIndex = std::make_shared<size_t>(0)](
                                             const std::vector<d5b::Vertex> &path, const d5b::SpectralVector &radiance) {
            if (!path.empty()) {
              image[y][x][0] += (path[0].localSurface.normal[0] * 0.5f + 0.5f);
              image[y][x][1] += (path[0].localSurface.normal[1] * 0.5f + 0.5f);
              image[y][x][2] += (path[0].localSurface.normal[2] * 0.5f + 0.5f);
            }
            return (*sampleIndex)++ > 16 ? d5b::Status::Done : d5b::Status::NotDone;
          };
          return problem;
        });
      }
    }
    return d5b::Status::Done;
  }

  float image[512][512][3]{};
};

class MyWorld final : public d5b::World {
public:
  [[nodiscard]] bool intersect(d5b::Random &, d5b::Ray ray, d5b::LocalSurface &localSurface) const override {
    mi::render::TriangleMesh::Location location;
    if (triangleMesh.rayTest(mi::Ray3f(ray.org, ray.dir, ray.minParam, ray.maxParam), location)) {
      localSurface.position = location.point;
      localSurface.texcoord = location.shading.texcoord;
      localSurface.tangents[0] = location.shading.tangents[0];
      localSurface.tangents[1] = location.shading.tangents[1];
      localSurface.normal = location.shading.normal;
      localSurface.scatteringProvider = [](const d5b::SpectralVector &, d5b::Scattering &scattering) {
        scattering.initializeLambertian(0.8, 0.0);
      };
      return true;
    }
    return false;
  }

  mi::render::TriangleMesh triangleMesh;
};

int main() {
  auto mesh = mi::geometry::Mesh(mi::geometry::FileOBJ("Suzanne.obj"));
  mesh.subdivide();
  mesh.calculateNormals();
  MySensor mySensor;
  MyWorld myWorld;
  myWorld.triangleMesh.buildFrom(mesh);
  d5b::Simulation simulation{};
  simulation.sensor = &mySensor;
  simulation.world = &myWorld;
  simulation.simulate();
  std::cout << "P3\n";
  std::cout << "512 512\n";
  std::cout << "255\n";
  for (size_t y = 0; y < 512; y++) {
    for (size_t x = 0; x < 512; x++) {
      std::cout << int(255 * (mySensor.image[y][x][0]) / 16) << ' ';
      std::cout << int(255 * (mySensor.image[y][x][1]) / 16) << ' ';
      std::cout << int(255 * (mySensor.image[y][x][2]) / 16) << ' ';
    }
    std::cout << std::endl;
  }
  return 0;
}
