#ifdef BUILD_CUDA

#ifndef FLUID_CUDA_H
#define FLUID_CUDA_H

#include <unordered_set>
#include <unordered_map>
#include <vector>
#include <memory>
#include <iomanip>
#include <iostream>

#include "cuNSearch.h"
#include "../real.h"
#include "../fluidParameters.h"
#include "../collision/collisionObject.h"

using namespace std;

class Fluid_cuda {
  public:
  Fluid_cuda(
    unique_ptr<vector<REAL3>> &&particle_positions,
    double h
  );
  ~Fluid_cuda();
  void init();
  void simulate(double frames_per_sec, double simulation_steps,
                const FluidParameters *cp,
                vector<CollisionObject *> *collision_objects);

  vector<REAL3> &getParticlePositions() {
    return *particle_positions;
  }

  const vector<REAL3> &getParticlePositions() const {
    return *particle_positions;
  }

  // __device__ void scramble();

  // Fluid properties

// private:
  // Fluid components
  // input
  unique_ptr<vector<REAL3>> particle_positions;
  // internal data
  cuNSearch::NeighborhoodSearch nsearch;
  int **neighbor_search_results_dev;
  int **neighbor_search_results_host;
  int *num_particles_dev;
  REAL3 *particle_positions_device;
  REAL3 *particle_velocities_device;
  REAL3 *particle_preditced_position_device;
  REAL3 *delta_p_device;
  REAL *lambda_device;
};


#endif
#endif
