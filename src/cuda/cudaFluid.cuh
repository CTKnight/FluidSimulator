#ifdef BUILD_CUDA

#ifndef FLUID_CUDA_H
#define FLUID_CUDA_H

#include <vector>
#include <memory>
#include <iomanip>
#include <iostream>

#include "cuNSearch.h"
#include "cudaUtils.h"
#include "../real.h"
#include "../fluidParameters.h"
#include "../collision/collisionObject.h"

using namespace std;

class Fluid_cuda {
  public:
  Fluid_cuda(
    unique_ptr<vector<REAL3>> &&particle_positions,
    REAL h
  );
  ~Fluid_cuda();
  void init();
  void simulate(REAL delta_t,
                const FluidParameters *cp,
                vector<CollisionObject *> *collision_objects);

  vector<REAL3> &getParticlePositions() {
    return *particle_positions;
  }

  const vector<REAL3> &getParticlePositions() const {
    return *particle_positions;
  }

private:

  void find_neighbors();
  // Fluid properties

  // Fluid components
  // input
  unique_ptr<vector<REAL3>> particle_positions;

  // internal data
  static constexpr int default_capacity{50};
  cuNSearch::NeighborhoodSearch nsearch;
  vector<int *> neighbor_search_results_host;
  vector<int> neighbor_search_results_size_host;
  vector<int> neighbor_search_results_capacity_host;
  int **neighbor_search_results_dev;
  int *neighbor_search_results_size_dev;

  REAL3 *particle_positions_device;
  REAL3 *particle_velocities_device;
  REAL3 *particle_predicted_positions_device;
  REAL3 *delta_p_device;
  REAL *lambda_device;
};


#endif
#endif
