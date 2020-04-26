#ifndef FLUID_H
#define FLUID_H

#include <unordered_set>
#include <unordered_map>
#include <vector>
#include <memory>

#include "CGL/CGL.h"
#include "CGL/misc.h"
#include <CompactNSearch>
#include "collision/collisionObject.h"

using namespace CGL;
using namespace std;

struct FluidParameters {
  FluidParameters(): FluidParameters(1000, 0.001, 0.1) {}
  FluidParameters(
    double density,
    double particle_mass,
    double damping,
    int solverIterations = 5,
    double particleRadius = 0.05)
      : density(density), particle_mass(particle_mass), damping(damping), solverIterations(solverIterations), particleRadius(particleRadius) {}
  ~FluidParameters() {}

  // Global simulation parameters

  double damping;
  double particle_mass;
  // unit: kg/m^3
  double density;
  int solverIterations;

  // render parameters

  // unit: m
  double particleRadius;
};

// default parameter: http://graphics.stanford.edu/courses/cs348c/PA1_PBF2016/index.html
struct Fluid {
  using Triad = array<CompactNSearch::Real, 3>;
  Fluid(
    unique_ptr<vector<Triad>> &&particle_positions, 
    unique_ptr<vector<Triad>> &&particle_velocities,
    double h,
    double epsilon = 600,
    double n = 4,
    double k = 0.0001,
    double c = 0.00007
  );
  ~Fluid() = default;
  void simulate(double frames_per_sec, double simulation_steps, const std::shared_ptr<FluidParameters> &cp,
                vector<Vector3D> external_accelerations,
                vector<CollisionObject *> *collision_objects);

  vector<Triad> &getParticlePositions() {
    return *particle_positions;
  }

  const vector<Triad> &getParticlePositions() const {
    return *particle_positions;
  }

  void reset();
  // Fluid properties

private:
  // Fluid components
  // input
  unique_ptr<vector<Triad>> particle_positions;
  unique_ptr<vector<Triad>> particle_velocities;
  // internal data
  vector<Triad> particle_preditced_positions;
  vector<Triad> delta_p;
  vector<double> lambda;
  double h;
  double epsilon;
  double n;
  double k;
  double c;

  CompactNSearch::NeighborhoodSearch nsearch;
  vector<vector<vector<unsigned int>>> neighbor_search_results;
};

inline Vector3D &triadAsVector3D(Fluid::Triad &triad);
inline const Vector3D &triadAsVector3D(const Fluid::Triad &triad);
inline vector<Vector3D> &triadAsVector3D(vector<Fluid::Triad> &triads);
inline const vector<Vector3D> &triadAsVector3D(const vector<Fluid::Triad> &triads);

inline double W_poly6(const Vector3D &r, double h) {
  const auto r2 = r.norm2();
  const auto h2 = pow(h, 2);
  if (r2 > h2) {
    return 0;
  }
  return 315 / (64 * PI * pow(h, 9)) * pow(h2 - r2, 3);
}

// https://www.wolframalpha.com/input/?i=gradient+15%2F%28PI*h%5E6%29*%28h-x%29%5E3
inline double W_spiky_gradient(const Vector3D &r_vec, double h) {
  const auto r = r_vec.norm();
  if (r > h) {
    return 0;
  }
  return -45 / (PI * pow(h, 6)) * pow(h - r, 2);
}

inline double W_viscosity(const Vector3D &r_vec, double h) {
  const auto r = r_vec.norm();
  if (r > h) {
    return 0;
  }
  return 15 / (2 * PI * pow(h, 3)) * ( -pow(r,3)/(2*pow(h,3)) + pow(r/h, 2) + (h/(2*r)) - 1);
}

std::istream& operator>>(std::istream& os, Vector3D& v);
std::istream& operator>>(std::istream& is, Fluid::Triad& v);
std::ostream& operator<<(std::ostream& os, const Fluid::Triad& v);
std::istream& operator>>(std::istream& is, Fluid& fluid);
std::ostream& operator<<(std::ostream& os, const Fluid& fluid);

#endif /* FLUID_H */