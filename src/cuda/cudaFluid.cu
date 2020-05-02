// #include <cuda>

#include "cudaFluid.cuh"

#ifdef BUILD_CUDA

__host__ __device__ void simulate_update_position() {

}

Fluid_cuda::Fluid_cuda(
  unique_ptr<vector<REAL3>> &&particle_positions,
  REAL h
): nsearch(h) {
  if (particle_positions == nullptr) {
    throw std::runtime_error("particle_positions == nullptr!");
  }
  this->particle_positions = std::move(particle_positions);
}

Fluid_cuda::~Fluid_cuda(){
  cudaFree(particle_positions_device);
  cudaFree(particle_velocities_device);
  cudaFree(particle_preditced_position_device);
  cudaFree(delta_p_device);
  cudaFree(lambda_device);
  cudaFree(num_particles_dev);
  const auto num_particle = particle_positions->size();

  for (int i = 0; i < num_particle; i++) {
    cudaFree(neighbor_search_results_host[i]);
  }
  delete[] neighbor_search_results_host;
  cudaFree(neighbor_search_results_dev);
}

void Fluid_cuda::init() {
  const auto num_particle = particle_positions->size();
  const auto SIZE_REAL3_N = sizeof(REAL3) * num_particle; 
  cudaMalloc(&particle_positions_device, SIZE_REAL3_N);

  cudaMalloc(&particle_velocities_device, SIZE_REAL3_N);
  cudaMemset(particle_velocities_device, 0, SIZE_REAL3_N);

  cudaMalloc(&particle_preditced_position_device, SIZE_REAL3_N);
  cudaMalloc(&delta_p_device, SIZE_REAL3_N);
  cudaMalloc(&lambda_device, sizeof(REAL)*num_particle);

  cudaMalloc(&num_particles_dev, sizeof(int));
  cudaMemcpy(num_particles_dev, &num_particle, sizeof(int), cudaMemcpyHostToDevice);

  cudaMalloc(&neighbor_search_results_dev, sizeof(int *) * num_particle);
  neighbor_search_results_host = new int*[num_particle];
  // size, capacity(include overheads of this 2 meta-elements)
  constexpr default_capacity = 50;
  int[] init_search_result = {0, default_capacity};
  for (int i = 0; i < num_particle; i++) {
    cudaMalloc(&neighbor_search_results_host[i], sizeof(int)*default_capacity);
    cudaMemcpy(neighbor_search_results_host[i], init_search_result, sizeof(int)*2, cudaMemcpyHostToDevice);
  }
  cudaMemcpy(neighbor_search_results_dev, neighbor_search_results_host, sizeof(int *) * num_particle, cudaMemcpyHostToDevice);
}

void Fluid_cuda::simulate(REAL frames_per_sec, REAL simulation_steps,
  const FluidParameters *cp,
  vector<CollisionObject *> *collision_objects) {
  
}

#endif