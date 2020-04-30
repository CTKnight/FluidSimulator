#ifndef COLLISIONOBJECT
#define COLLISIONOBJECT

#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "../real.h"
#include "../vector3R.h"

using namespace CGL;
using namespace std;
using namespace nanogui;

class CollisionObject {
public:
  virtual void render(GLShader &shader) = 0;
  virtual void collide(PointMass &pm) = 0;
  virtual void collide(Vector3R &position, Vector3R &delta_p) = 0;

private:
  REAL friction;
};

#endif /* COLLISIONOBJECT */
