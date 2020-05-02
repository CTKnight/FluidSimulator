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

#include "collision/collisionObject.h"
#include "marchingCube.h"
#include "real.h"
#include "vector3R.h"
#include "fluidParameters.h"

using namespace CGL;
using namespace std;

// default parameter: http://graphics.stanford.edu/courses/cs348c/PA1_PBF2016/index.html
struct Fluid {
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

#endif /* FLUID_H */