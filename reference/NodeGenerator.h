#pragma once

#include <array>
#include <random>

#include "dirsig5/common/Geometry.h"
#include "dirsig5/common/SpectralVector.h"
#include "dirsig5/internal/core/components/Scene.h"

namespace d5::core {

struct Problem;
struct Node;

/**
 * This function's purpose is to take a ray that has been intersected
 * with the scene and to construct a node with it. A return value of
 * "false" indicated that the path finished early (e.g. absorption)/
 */
class NodeGenerator {
public:
  /**
   * Generate a node from a ray.
   */
  static bool generateNode(Problem &problem, Node &node, size_t nWavelengths, size_t refWavelength);

  static void handleMediumTransition(Problem &problem, Node &node, size_t nWavelengths);

private:
  /**
   * Define some scratch space that we'll use locally.
   */
  struct Scratch {
    void update(size_t _nWavelengths) {
      if (nWavelengths != _nWavelengths) {
        sv[0].resize(_nWavelengths);
        nWavelengths = _nWavelengths;
      }
    }

    std::array<SpectralVector, 1> sv;
    Quaternionf quaternion;
    std::array<Vectorf, 3> vc;

    size_t nWavelengths = 0;
  };
};

} // namespace d5::core
