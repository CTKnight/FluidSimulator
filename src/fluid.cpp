#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "fluid.h"
#include "collision/plane.h"
#include "collision/sphere.h"

using namespace std;

Fluid::Fluid(unique_ptr<vector<Triad>> &&particle_positions): Fluid(std::move(particle_positions), nullptr) {}

Fluid::Fluid(
    unique_ptr<vector<Triad>> &&particle_positions, 
    unique_ptr<vector<Triad>> &&particle_velocities
): particleSphereMesh(4, 4), nsearch(0.01) {
  if (particle_positions == nullptr) {
    throw std::exception("particle_positions == nullptr!");
  }
  this->particle_positions  = std::move(particle_positions);
  if (particle_velocities != nullptr) {
    if (particle_positions->size() != particle_velocities->size()) {
      throw std::exception("particle_positions->size()  != particle_velocities->size()!");
    }
    this->particle_velocities = std::move(particle_velocities);
  } else {
    auto velocities = make_unique<vector<Triad>>();
    velocities->resize(this->particle_positions->size());
    std::array<CompactNSearch::Real, 3> defaultVelocity = {0.0, 0.0, 0.0};
    std::fill(velocities->begin(), velocities->end(), defaultVelocity);
    this->particle_velocities = std::move(velocities);
  }
  // TODO: Init nsearch
}

void Fluid::simulate(double frames_per_sec, double simulation_steps, const std::shared_ptr<FluidParameters> &cp,
                vector<Vector3D> external_accelerations,
                vector<CollisionObject *> *collision_objects) {

}

void Fluid::render(nanogui::GLShader &shader, const std::shared_ptr<FluidParameters> &cp) {
  Vector3D position;
  for (const auto &p: *this->particle_positions) {
    position[0] = p[0];
    position[1] = p[1];
    position[2] = p[2];
    particleSphereMesh.draw_sphere(shader, position, cp->particleRadius);
  }
}

void Fluid::reset() {

}