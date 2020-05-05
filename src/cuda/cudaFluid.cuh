#ifdef BUILD_CUDA

#ifndef FLUID_CUDA_H
#define FLUID_CUDA_H

#include <vector>
#include <memory>
#include <iomanip>
#include <iostream>
#include <thrust/device_vector.h>

#include "cuNSearch.h"
#include "cudaUtils.h"
#include "../real.h"
#include "../fluidParameters.h"
#include "cudaPlane.cuh"

using namespace std;

class Fluid_cuda {
  public:
  Fluid_cuda(
    unique_ptr<vector<REAL3>> &&particle_positions,
    unique_ptr<vector<REAL3>> &&particle_velocities,
    REAL h
  );
  ~Fluid_cuda();
  void init();
  void simulate(REAL delta_t,
                const FluidParameters *cp,
                thrust::device_vector<Plane_cuda> &collision_objects);

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
  unique_ptr<vector<REAL3>> particle_velocities;

  // internal data
  static constexpr int default_capacity{30};
  cuNSearch::NeighborhoodSearch nsearch;
  vector<int> neighbor_search_results_host;
  vector<int> neighbor_search_results_size_prefix_sum_host;
  int neighbor_search_results_dev_capacity;
  int *neighbor_search_results_dev;
  int *neighbor_search_results_size_prefix_sum_dev;

  REAL3 *particle_positions_device;
  REAL3 *particle_velocities_device;
  REAL3 *particle_predicted_positions_device;
  REAL3 *delta_p_device;
  REAL *lambda_device;
};


#endif
#endif
