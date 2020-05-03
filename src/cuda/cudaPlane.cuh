#ifndef CUDA_PLANE_H
#define CUDA_PLANE_H

#include "../real.h"
#include "../vector3R.h"
#include "../cuda/cudaUtils.h"
#include "../collision/plane.h"

struct Plane_cuda {
public:
  Plane_cuda() = default;
  Plane_cuda(const Plane& p): Plane_cuda(p.point, p.normal, p.friction) {}
  Plane_cuda(const Vector3R &point, const Vector3R &normal, double friction)
      : point(point), normal(normal.unit()), friction(friction) {}

  CUDA_CALLABLE void collide(Vector3R &position, Vector3R &delta_p) {
    const auto &predicted_position = position + delta_p;
    // TODO: assumption: axis aligned
    for (int i = 0; i < 3; i++) {
      const auto diff = predicted_position[i]-point[i];
      if (normal[i] == 1) {
        if (diff < 0) {
          // delta_p[i] = point[i] - diff - position[i];
          delta_p[i] = point[i]- position[i];
        }
        return;
      } else if (normal[i] == -1) {
        if (diff > 0) {
          // delta_p[i] = point[i] - diff - position[i];
          delta_p[i] = point[i]- position[i];
        }
        return;
      }
    }
  }

  Vector3R point;
  Vector3R normal;

  REAL friction;
};

#endif