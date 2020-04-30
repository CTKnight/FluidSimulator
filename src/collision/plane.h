#ifndef COLLISIONOBJECT_PLANE_H
#define COLLISIONOBJECT_PLANE_H

#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "collisionObject.h"

using namespace nanogui;
using namespace CGL;
using namespace std;

struct Plane : public CollisionObject {
public:
  Plane(const Vector3R &point, const Vector3R &normal, double friction)
      : point(point), normal(normal.unit()), friction(friction) {}

  void render(GLShader &shader);
  void collide(PointMass &pm);
  void collide(Vector3R &position, Vector3R &delta_p);

  Vector3R point;
  Vector3R normal;

  double friction;
};

#endif /* COLLISIONOBJECT_PLANE_H */
