#include "dirsig5b/Simulator.h"

namespace d5b {

void Simulator::simulate() {
  std::vector<SensorRay> sensorRays;
  while (true) {
    sensorRays.reserve(512);
    sensorRays.clear();
    Sensor::Status status{sensor->populateSensorRays(sensorRays)};
#pragma omp parallel
    for (const SensorRay &sensorRay : sensorRays) {
      simulate(sensorRay);
    }
    if (status == Sensor::Status::Done) break;
  }
}

void Simulator::simulate(const SensorRay &sensorRay) {
  //
}

} // namespace d5b
