#include <iostream>
#include <math.h>
#include <random>
#include <vector>
#include <exception>
#include <omp.h>

#include "fluid.h"
#include "collision/plane.h"
#include "collision/sphere.h"

using namespace std;

Fluid::Fluid(unique_ptr<vector<Triad>> &&particle_positions): Fluid(std::move(particle_positions), nullptr) {}

Fluid::Fluid(
    unique_ptr<vector<Triad>> &&particle_positions, 
    unique_ptr<vector<Triad>> &&particle_velocities
): nsearch(0.01) {
  if (particle_positions == nullptr) {
    throw std::runtime_error("particle_positions == nullptr!");
  }
  const auto n = particle_positions->size();
  this->particle_positions  = std::move(particle_positions);
  if (particle_velocities != nullptr) {
    if (n != particle_velocities->size()) {
      throw std::runtime_error("particle_positions->size()  != particle_velocities->size()!");
    }
    this->particle_velocities = std::move(particle_velocities);
  } else {
    auto velocities = make_unique<vector<Triad>>();
    velocities->resize(this->particle_positions->size());
    // Init to 0
    memset(velocities->data(), 0, sizeof(Fluid::Triad)*n);
    this->particle_velocities = std::move(velocities);
  }
  this->particle_preditced_positions.resize(n);
  this->delta_p.resize(n);
  this->lambda.resize(n);

  // TODO: Init nsearch
}

void Fluid::simulate(double frames_per_sec, double simulation_steps, const std::shared_ptr<FluidParameters> &cp,
                vector<Vector3D> external_accelerations,
                vector<CollisionObject *> *collision_objects) {
  double delta_t = 1.0f / frames_per_sec / simulation_steps;
  auto &particle_positions = triadAsVector3D(*this->particle_positions);
  const auto n = particle_positions.size();
  auto &particle_velocities = triadAsVector3D(*this->particle_velocities);
  auto &preditced_positions = triadAsVector3D(this->particle_preditced_positions);

  #pragma omp parallel for num_threads(4) 
  for (int i = 0; i < n; i++) {
    for (const auto &acc: external_accelerations) {
      particle_velocities[i] += acc * delta_t;
    }
    preditced_positions[i] = particle_positions[i] + particle_velocities[i] * delta_t;
  }
  memcpy(particle_positions.data(), preditced_positions.data(), sizeof(Fluid::Triad)*n);
}

void Fluid::reset() {

}

inline Vector3D &triadAsVector3D(Fluid::Triad &triad) {
  return reinterpret_cast<Vector3D &>(triad);
}

inline vector<Vector3D> &triadAsVector3D(vector<Fluid::Triad> &triads) {
  return reinterpret_cast<vector<Vector3D> &>(triads);
}