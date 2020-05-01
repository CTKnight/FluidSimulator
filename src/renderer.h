#ifndef RENDERER_H
#define RENDERER_H

#include <nanogui/nanogui.h>
#include "fluid.h"
#include "collision/collisionObject.h"
#include "collision/plane.h"
#include "collision/sphere.h"
#include "misc/sphere_drawing.h"

using namespace nanogui;

struct ObjectRenderer {
  ObjectRenderer() = default;
  virtual void render(GLShader &shader, const Fluid &fluid, const FluidParameters &fp) = 0;
  virtual void render(GLShader &shader, const vector<CollisionObject *> &collision_objects) = 0;
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

  void render(GLShader &shader, const vector<CollisionObject *> &collision_objects) {
    for (CollisionObject *co : collision_objects) {
      if (Plane *p = dynamic_cast<Plane *>(co)) {
        nanogui::Color color(0.7f, 0.7f, 0.7f, 1.0f);
        const auto &point = p->point;
        const auto &normal = p->normal;
        Vector3f sPoint(point.x, point.y, point.z);
        Vector3f sNormal(normal.x, normal.y, normal.z);
        Vector3f sParallel(normal.y - normal.z, normal.z - normal.x,
                          normal.x - normal.y);
        sParallel.normalize();
        Vector3f sCross = sNormal.cross(sParallel);

        MatrixXf positions(3, 4);
        MatrixXf normals(3, 4);

        positions.col(0) << sPoint + 2 * (sCross + sParallel);
        positions.col(1) << sPoint + 2 * (sCross - sParallel);
        positions.col(2) << sPoint + 2 * (-sCross + sParallel);
        positions.col(3) << sPoint + 2 * (-sCross - sParallel);

        normals.col(0) << sNormal;
        normals.col(1) << sNormal;
        normals.col(2) << sNormal;
        normals.col(3) << sNormal;

        if (shader.uniform("u_color", false) != -1) {
          shader.setUniform("u_color", color);
        }
        shader.uploadAttrib("in_position", positions);
        if (shader.attrib("in_normal", false) != -1) {
          shader.uploadAttrib("in_normal", normals);
        }

        shader.drawArray(GL_TRIANGLE_STRIP, 0, 4);
      } else if (Sphere *p = dynamic_cast<Sphere *>(co)) {
        particleSphereMesh.draw_sphere(shader, p->origin, p->radius * 0.92);
      }
    }
  }

  private:
  Misc::SphereMesh particleSphereMesh;
};

#endif
