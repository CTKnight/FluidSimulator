#ifndef COLLISIONOBJECT
#define COLLISIONOBJECT

#include "../real.h"
#include "../vector3R.h"
#include "../cuda/cudaUtils.h"


class CollisionObject {
public:
  CUDA_CALLABLE virtual void collide(Vector3R &position, Vector3R &delta_p) = 0;

private:
  REAL friction;
};

#endif /* COLLISIONOBJECT */
