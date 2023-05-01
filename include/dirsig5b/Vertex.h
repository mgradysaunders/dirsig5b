#pragma once

#include "dirsig5b/LocalSurface.h"
#include "dirsig5b/LocalVolume.h"

namespace d5b {

class D5B_API Vertex final {
public:
  /// The kind of vertex.
  enum class Kind {
    Sensor,  ///< The initial vertex.
    Surface, ///< An interior vertex to represent scattering at a surface.
    Volume,  ///< An interior vertex to represent scattering within a volume.
    Light    ///< The terminal vertex.
  };

  /// The kind of vertex.
  Kind kind{};

  /// The position.
  Vector3 position;

  /// The local-to-world orthonormal basis.
  Matrix3 localToWorld{Matrix3::identity()};

  /// The outgoing direction (back to camera).
  Vector3 pathOmegaO;

  /// The incident direction.
  Vector3 pathOmegaI;

  /// If kind is Surface, the local surface description.
  LocalSurface localSurface{};

  /// If kind is Volume, the local volume description.
  LocalVolume localVolume{};

  /// The scattering functions. Only present if kind is Surface or Volume, but may
  /// not be present even if kind is Surface or Volume, in which case the understood
  /// behavior is to scattering like a delta in the direction of propagation.
  std::optional<Scattering> scattering{};

  void scatterBSDF(Vector3 omegaO, Vector3 omegaI, SpectralVector &f) const {
    f = scattering->scatterBSDF(dot(omegaO, localToWorld), dot(omegaI, localToWorld));
  }

  [[nodiscard]] double scatterPDF(Vector3 omegaO, Vector3 omegaI) const {
    return scattering->scatterPDF(dot(omegaO, localToWorld), dot(omegaI, localToWorld));
  }

  [[nodiscard]] double importanceSample(Random &random, Vector3 omegaO, Vector3 &omegaI, SpectralVector &throughput) const {
    omegaO = dot(omegaO, localToWorld);
    double p = scattering->scatterImportanceSample(random, omegaO, omegaI, throughput);
    omegaI = dot(localToWorld, omegaI);
    return p;
  }

  /// The throughput at this node, cumulative product of BSDF over density.
  SpectralVector pathThroughput;
};

using Path = std::vector<Vertex>;

} // namespace d5b
