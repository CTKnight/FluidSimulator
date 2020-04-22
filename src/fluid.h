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
  FluidParameters() {}
  FluidParameters(
    double damping,
    double density,
    double ks,
    double particleRadius = 0.05)
      :damping(damping), density(density), ks(ks), particleRadius(particleRadius) {}
  ~FluidParameters() {}

  // Global simulation parameters


  double damping;

  // Mass-spring parameters
  double density;
  double ks;

  // render parameters
  double particleRadius;
};

struct Fluid {
  using Triad = array<CompactNSearch::Real, 3>;
  Fluid(unique_ptr<vector<Triad>> &&particle_positions);
  Fluid(
    unique_ptr<vector<Triad>> &&particle_positions, 
    unique_ptr<vector<Triad>> &&particle_velocities
  );
  ~Fluid() = default;
  void simulate(double frames_per_sec, double simulation_steps, const std::shared_ptr<FluidParameters> &cp,
                vector<Vector3D> external_accelerations,
                vector<CollisionObject *> *collision_objects);

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
inline vector<Vector3D> &triadAsVector3D(vector<Fluid::Triad> &triads);

#endif /* FLUID_H */