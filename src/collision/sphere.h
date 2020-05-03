#ifndef COLLISIONOBJECT_SPHERE_H
#define COLLISIONOBJECT_SPHERE_H

#include "collisionObject.h"

using namespace std;

struct Sphere : public CollisionObject {
public:
  Sphere(const Vector3R &origin, REAL radius, REAL friction)
      : origin(origin), radius(radius), radius2(radius * radius),
        friction(friction) {}

  CUDA_CALLABLE void collide(Vector3R &position, Vector3R &delta_p);

  Vector3R origin;
  REAL radius;
  REAL radius2;

private:

  REAL friction;
};

#endif /* COLLISIONOBJECT_SPHERE_H */
