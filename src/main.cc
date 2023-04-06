#include "dirsig5b/common.h"

#include <Microcosm/Geometry/FileOBJ>
#include <Microcosm/Geometry/Mesh>
#include <Microcosm/Render/TriangleMesh>

#include <iostream>

#if 0
class MySensor final : public d5::Sensor {
public:
};

class MyWorld final : public d5::World {
public:
  [[nodiscard]] bool intersect(const d5::Ray &ray, d5::Intersection *intersection = nullptr) const {
    mi::render::TriangleMesh::Location location;
    if (triangleMesh.rayTest(mi::Ray3f(ray.org, ray.dir, ray.minParam, ray.maxParam), location)) {
      if (intersection) {
        intersection->position = location.point;
        intersection->texcoord = location.shading.texcoord;
        intersection->getScattering =
          [transform = d5::Transform(d5::Matrix3::orthonormalBasis(location.shading.normal))](const d5::SpectralVector &) {
            return d5::Scattering::AddTransform(transform, d5::Scattering::Lambertian(0.8, 0.0));
          };
      }
      return true;
    }
    return false;
  }

  mi::render::TriangleMesh triangleMesh;
};
#endif

int main() {
#if 0
  d5::PathTracer pathTracer{};
  MySensor mySensor;
  MyWorld myWorld;
  myWorld.triangleMesh.buildFrom(mi::geometry::Mesh(mi::geometry::FileOBJ("Suzanne.obj")));
  d5::Random random;
  pathTracer.sensor = &mySensor;
  pathTracer.world = &myWorld;
  pathTracer.wavelengths = {0.4, 0.5, 0.6};
  auto nodes = pathTracer.generatePath(random, d5::Ray{{0, 0, -2}, {0, 0, 1}}, {1.0, 1.0, 1.0});
  std::cout << nodes.size() << std::endl;
#endif
  return 0;
}
