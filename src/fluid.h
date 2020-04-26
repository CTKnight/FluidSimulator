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
#include "marchingCube.h"

using namespace CGL;
using namespace std;

struct FluidParameters {
  FluidParameters(): FluidParameters(1000, 0.001, 0.1) {}
  FluidParameters(
    double density,
    double particle_mass,
    double damping,
    double particleRadius = 0.05)
      : density(density), particle_mass(particle_mass), damping(damping), particleRadius(particleRadius) {}
  ~FluidParameters() {}

  // Global simulation parameters

  double damping;
  double particle_mass;
  // unit: kg/m^3
  double density;

  // render parameters

  // unit: m
  double particleRadius;
};

struct Fluid {
  using Triad = array<CompactNSearch::Real, 3>;
  Fluid(unique_ptr<vector<Triad>> &&particle_positions);
  Fluid(
    unique_ptr<vector<Triad>> &&particle_positions, 
    unique_ptr<vector<Triad>> &&particle_velocities,
    double h = 0.01
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
  vector<Triad> lambda;

  CompactNSearch::NeighborhoodSearch nsearch;
};

inline Vector3D &triadAsVector3D(Fluid::Triad &triad);
inline const Vector3D &triadAsVector3D(const Fluid::Triad &triad);
inline vector<Vector3D> &triadAsVector3D(vector<Fluid::Triad> &triads);
inline const vector<Vector3D> &triadAsVector3D(const vector<Fluid::Triad> &triads);

std::istream& operator>>(std::istream& os, Vector3D& v);
std::istream& operator>>(std::istream& is, Fluid::Triad& v);
std::ostream& operator<<(std::ostream& os, const Fluid::Triad& v);
std::istream& operator>>(std::istream& is, Fluid& fluid);
std::ostream& operator<<(std::ostream& os, const Fluid& fluid);

#endif /* FLUID_H */