#ifndef FLUID_H
#define FLUID_H

#include <unordered_set>
#include <unordered_map>
#include <vector>
#include <memory>

#ifdef BUILD_CUDA
#include "cuNSearch.h"
#else
#include <CompactNSearch>
#endif

#include "CGL/CGL.h"
#include "CGL/misc.h"
#include "collision/collisionObject.h"
#include "marchingCube.h"
#include "real.h"
#include "vector3R.h"

using namespace CGL;
using namespace std;

struct FluidParameters {
  FluidParameters()=default;
  FluidParameters(const FluidParameters& other)=default;
  FluidParameters(
    REAL density,
    REAL particle_mass,
    REAL damping,
    REAL h,
    REAL epsilon,
    REAL n,
    REAL k,
    REAL c,
    int solverIterations = 4,
    REAL particleRadius = 0.02)
      : density(density), particle_mass(particle_mass), damping(damping),
        h(h), epsilon(epsilon), n(n), k(k), c(c),
        solverIterations(solverIterations), particleRadius(particleRadius) {}
  ~FluidParameters() {}

  // Global simulation parameters
  Vector3R external_forces;
  REAL damping;
  REAL particle_mass;
  REAL h;
  REAL epsilon;
  REAL n;
  REAL k;
  REAL c;
  // unit: kg/m^3
  REAL density;
  int solverIterations;

  // render parameters

  // unit: m
  REAL particleRadius;
};

// default parameter: http://graphics.stanford.edu/courses/cs348c/PA1_PBF2016/index.html
struct Fluid {
  using REAL3 = array<REAL, 3>;
  Fluid(
    unique_ptr<vector<REAL3>> &&particle_positions, 
    unique_ptr<vector<REAL3>> &&particle_velocities,
    REAL h
  );
  ~Fluid() = default;
  void init();
  void simulate(REAL frames_per_sec, REAL simulation_steps,
                const std::shared_ptr<FluidParameters> &cp, 
                vector<CollisionObject *> *collision_objects);

  vector<REAL3> &getParticlePositions() {
    return *particle_positions;
  }

  const vector<REAL3> &getParticlePositions() const {
    return *particle_positions;
  }

  void reset();
  // Fluid properties

private:
  // Fluid components
  // input
  unique_ptr<vector<REAL3>> particle_positions;
  unique_ptr<vector<REAL3>> particle_velocities;
  // internal data
  vector<REAL3> particle_preditced_positions;
  vector<REAL3> delta_p;
  vector<REAL> lambda;

  #ifdef BUILD_CUDA
  cuNSearch::NeighborhoodSearch nsearch;
  #else
  CompactNSearch::NeighborhoodSearch nsearch;
  #endif
  vector<vector<vector<unsigned int>>> neighbor_search_results;
};

inline Vector3R &REAL3AsVector3R(Fluid::REAL3 &REAL3);
inline const Vector3R &REAL3AsVector3R(const Fluid::REAL3 &REAL3);
inline vector<Vector3R> &REAL3AsVector3R(vector<Fluid::REAL3> &REAL3s);
inline const vector<Vector3R> &REAL3AsVector3R(const vector<Fluid::REAL3> &REAL3s);

inline REAL W_poly6(const Vector3R &r, REAL h) {
  const auto r2 = r.norm2();
  const auto h2 = pow(h, 2);
  if (r2 > h2) {
    return 0;
  }
  return 315 / (64 * PI * pow(h, 9)) * pow(h2 - r2, 3);
}

// https://www.wolframalpha.com/input/?i=gradient+15%2F%28PI*h%5E6%29*%28h-x%29%5E3
inline REAL W_spiky_gradient(const Vector3R &r_vec, REAL h) {
  const auto r = r_vec.norm();
  if (r > h) {
    return 0;
  }
  return -45 / (PI * pow(h, 6)) * pow(h - r, 2);
}

inline REAL W_viscosity(const Vector3R &r_vec, REAL h) {
  const auto r = r_vec.norm();
  if (r > h) {
    return 0;
  }
  return 15 / (2 * PI * pow(h, 3)) * ( -pow(r,3)/(2*pow(h,3)) + pow(r/h, 2) + (h/(2*r)) - 1);
}

std::istream& operator>>(std::istream& is, Fluid::REAL3& v);
std::ostream& operator<<(std::ostream& os, const Fluid::REAL3& v);
std::istream& operator>>(std::istream& is, Fluid& fluid);
std::ostream& operator<<(std::ostream& os, const Fluid& fluid);

#endif /* FLUID_H */