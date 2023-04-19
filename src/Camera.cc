#include "Camera.h"

#include <Microcosm/stbi>
#include <Microcosm/utility>
#include <iostream>

void Camera::initialize() {
  cache.seedSequence = d5b::Random(seed);
  cache.wavelength.resize(wavelengthCount);
  cache.throughput.resize(wavelengthCount);
  cache.wavelength = 0;
  cache.throughput = 1;
  for (size_t i = 0; i < wavelengthCount; i++)
    cache.wavelength[i] = mi::lerp(double(i) / double(wavelengthCount - 1), wavelengthMin, wavelengthMax);
  cache.numSamples = 0;
  cache.aspectRatio = double(sizeX) / double(sizeY);
  cache.focalLength = 1.0 / (2.0 * mi::tan(fovY / 2.0));
  buffer.resize(sizeY, sizeX, wavelengthCount);
  buffer = 0;
}

void Camera::request(std::vector<d5b::ProblemRequest> &problemRequests) {
  problemRequests.reserve(sizeX * sizeY);
  for (size_t pixelY = 0; pixelY < sizeY; pixelY++)
    for (size_t pixelX = 0; pixelX < sizeX; pixelX++)
      problemRequests.emplace_back([this, pixelX, pixelY, problemSeed = cache.seedSequence()] {
        d5b::Problem problem;
        problem.seed = problemSeed;
        problem.maxBounces = this->maxBounces;
        problem.wavelength = this->cache.wavelength;
        problem.throughput = this->cache.throughput;
        problem.sampleRay = [this, pixelX, pixelY](d5b::Random &random) -> d5b::Ray {
          double uX = d5b::randomize<double>(random);
          double uY = d5b::randomize<double>(random);
          d5b::Ray ray{};
          ray.dir[0] = ((pixelX + uX) / double(this->sizeX) - 0.5) * this->cache.aspectRatio;
          ray.dir[1] = ((pixelY + uY) / double(this->sizeY) - 0.5) * -1;
          ray.dir[2] = -this->cache.focalLength;
          ray.dir = mi::fastNormalize(ray.dir);
#if 0
          if (this->dofRadius > 0) {
            d5b::Vector3 focus = ray.org + this->dofDistance / mi::abs(ray.dir[2]) * ray.dir;
            d5b::Vector2 shift;
            if (this->dofApertureBlades < 3) {
              shift = mi::render::uniformDiskSample(random);
            } else {
              size_t t = random(this->dofApertureBlades - 2) + 1;
              double angleB = 2 * d5b::Pi * double(t + 0) / double(this->dofApertureBlades);
              double angleC = 2 * d5b::Pi * double(t + 1) / double(this->dofApertureBlades);
              mi::Triangle2d triangle = {
                mi::Vector2d(1, 0),                             //
                mi::Vector2d(mi::cos(angleB), mi::sin(angleB)), //
                mi::Vector2d(mi::cos(angleC), mi::sin(angleC))};
              shift = triangle.areaSample(random).point;
            }
            ray.org[0] += this->dofRadius * shift[0];
            ray.org[1] += this->dofRadius * shift[1];
            ray.dir = mi::normalize(focus - ray.org);
          }
#endif
          return ray.withTransform(localToWorld);
        };
        struct AcceptPathContribution {
          Camera *self{};
          size_t pixelX{};
          size_t pixelY{};
          size_t numSamples{};
          [[nodiscard]] d5b::Status operator()(const std::vector<d5b::Vertex> &, const d5b::SpectralVector &radiance) {
            for (size_t i = 0; i < self->wavelengthCount; i++) {
              self->buffer(pixelY, pixelX, i) += radiance[i];
            }
            return ++numSamples >= self->numSamplesPerBatch ? d5b::Status::Done : d5b::Status::NotDone;
          }
        };
        problem.acceptPathContribution = AcceptPathContribution{this, pixelX, pixelY, 0};
        return problem;
      });
}

d5b::Status Camera::finish() {
  cache.numSamples += numSamplesPerBatch;
  using namespace mi::string_literals;
  std::cout << "Finished {}/{} samples!\n"_format(cache.numSamples, maxSamples);
  std::cout.flush();
  mi::stbi::ImageU8 imageU8(mi::with_shape, sizeY, sizeX, 3);
#pragma omp parallel for num_threads(100) schedule(dynamic)
  for (size_t pixelY = 0; pixelY < sizeY; pixelY++) {
    for (size_t pixelX = 0; pixelX < sizeX; pixelX++) {
      mi::Vector3d color;
      double coefficient = (wavelengthMax - wavelengthMin) / wavelengthCount / cache.numSamples;
      for (size_t i = 0; i < wavelengthCount; i++) {
        double wavelength = cache.wavelength[i];
        double radiance = coefficient * buffer(pixelY, pixelX, i);
        color[0] += mi::wymanFit1931X(wavelength) * radiance;
        color[1] += mi::wymanFit1931Y(wavelength) * radiance;
        color[2] += mi::wymanFit1931Z(wavelength) * radiance;
      }
      color = mi::convertXYZToRGB(color);
      color = mi::encodeSRGB(mi::tonemapACES(mi::Vector3d(gain * color)));
      imageU8(pixelY, pixelX, mi::Slice()).assign(255 * color);
    }
  }
  mi::stbi::saveU8(mi::stbi::FileType::PNG, basename + ".png", imageU8);
  return cache.numSamples >= maxSamples ? d5b::Status::Done : d5b::Status::NotDone;
}
