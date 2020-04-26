#include <iomanip>
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
    unique_ptr<vector<Triad>> &&particle_velocities,
    double h
): nsearch(h) {
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

inline const Vector3D &triadAsVector3D(const Fluid::Triad &triad) {
  return reinterpret_cast<const Vector3D &>(triad);
}

inline vector<Vector3D> &triadAsVector3D(vector<Fluid::Triad> &triads) {
  return reinterpret_cast<vector<Vector3D> &>(triads);
}

inline const vector<Vector3D> &triadAsVector3D(const vector<Fluid::Triad> &triads) {
  return reinterpret_cast<const vector<Vector3D> &>(triads);
}

std::istream& operator>>(std::istream& is, Vector3D& v) {
  string buffer;
  char c;
  std::getline(is, buffer);
  std::stringstream ss(buffer);
  ss >> c >> v.x >> c >> v.y >> c >> v.z;
  return is;
}

std::istream& operator>>(std::istream& is, Fluid::Triad& v) {
  is >> v[0] >> v[1] >> v[2];
  return is;
}

std::ostream& operator<<(std::ostream& os, const Fluid::Triad& v) {
  os << v[0] << ' ' << v[1] << ' ' << v[2];
  return os;
}

std::istream& operator>>(std::istream& is, Fluid& fluid) {
  string placeholder;
  int n;
  auto &particles = fluid.getParticlePositions();
  for (int i = 0; i < 4; i++) {
    std::getline(is, placeholder);
  }
  is >> placeholder >> n >> placeholder;
  std::getline(is, placeholder);
  particles.resize(n);
  for (int i = 0; i < n; i++) {
    is >> particles[i];
  }
  return is;
}

std::ostream& operator<<(std::ostream& os, const Fluid& fluid) {
  const auto &particles = fluid.getParticlePositions();
  // VTK headers
  os << "# vtk DataFile Version 3.0\n";
  os << "vtk output\n";
  os << "ASCII\n";
  os << "DATASET UNSTRUCTURED_GRID\n";
  os << "POINTS " << particles.size() << " float\n";
  os << std::setprecision(5) << std::fixed;
  for (const auto &p: particles) {
    os << p << "\n";
  }
  return os;
}