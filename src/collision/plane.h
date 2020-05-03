#ifndef COLLISIONOBJECT_PLANE_H
#define COLLISIONOBJECT_PLANE_H

#include "collisionObject.h"

using namespace std;

struct Plane : public CollisionObject {
public:
  Plane() = default;
  Plane(const Vector3R &point, const Vector3R &normal, double friction)
      : point(point), normal(normal.unit()), friction(friction) {}

  CUDA_CALLABLE void collide(Vector3R &position, Vector3R &delta_p);

  Vector3R point;
  Vector3R normal;

  REAL friction;
};

#endif /* COLLISIONOBJECT_PLANE_H */
