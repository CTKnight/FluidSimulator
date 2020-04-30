#ifndef RENDERER_H
#define RENDERER_H

#include <nanogui/nanogui.h>
#include "fluid.h"
#include "collision/collisionObject.h"
#include "misc/sphere_drawing.h"

using namespace nanogui;

struct ObjectRenderer {
  ObjectRenderer() = default;
  virtual void render(GLShader &shader, const Fluid &fluid, const FluidParameters &fp) = 0;
  virtual void render(GLShader &shader, const vector<CollisionObject *> &collision_objects) {
    for (CollisionObject *co : collision_objects) {
      co->render(shader);
    }
  }
};

struct OpenGLRenderder: ObjectRenderer {
  OpenGLRenderder(const Misc::SphereMesh &sphereMesh): ObjectRenderer(), particleSphereMesh(sphereMesh)  {

  }

  void render(GLShader &shader, const Fluid &fluid, const FluidParameters &fp) override {
    const auto &particlePositions = fluid.getParticlePositions();
    Vector3R position;
    for (const auto &p: particlePositions) {
      position[0] = p[0];
      position[1] = p[1];
      position[2] = p[2];
      particleSphereMesh.draw_sphere(shader, position, fp.particleRadius);
    }
  }

  private:
  Misc::SphereMesh particleSphereMesh;
};

#endif
