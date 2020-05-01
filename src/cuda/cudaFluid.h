#ifdef BUILD_CUDA

#ifndef FLUID_CUDA_H
#define FLUID_CUDA_H

#include <unordered_set>
#include <unordered_map>
#include <vector>
#include <memory>

#include "cuNSearch.h"
#include "../real.h"
#include "../fluidParameters.h"

using namespace std;

struct Fluid_cuda {
  Fluid_cuda(
    unique_ptr<vector<REAL3>> &&particle_positions, 
    unique_ptr<vector<REAL3>> &&particle_velocities,
    double h
  );
  ~Fluid_cuda() = default;
  void init();
  void simulate(double frames_per_sec, double simulation_steps
  ,             const std::shared_ptr<FluidParameters> &cp
                // , 
                // vector<CollisionObject *> *collision_objects
                );

  vector<REAL3> &getParticlePositions() {
    return *particle_positions;
  }

  const vector<REAL3> &getParticlePositions() const {
    return *particle_positions;
  }

  void reset();
  // Fluid properties

// private:
  // Fluid components
  // input
  unique_ptr<vector<REAL3>> particle_positions;
  unique_ptr<vector<REAL3>> particle_velocities;
  // internal data
  vector<REAL3> particle_preditced_positions;
  vector<REAL3> delta_p;
  vector<double> lambda;

  CompactNSearch::NeighborhoodSearch nsearch;
  vector<vector<vector<unsigned int>>> neighbor_search_results;

  double *particle_positions_device;
  double *particle_velocities_device;
  double *particle_preditced_position_device;
  double *delta_p_device;
  REAL *lambda_device;
};

#endif
#endif
