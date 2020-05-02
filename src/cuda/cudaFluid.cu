#include "cudaFluid.cuh"
#include <cmath>

#ifdef BUILD_CUDA

inline __host__ __device__ REAL W_poly6_cuda(const Vector3R &r, REAL h) {
  const auto r2 = r.norm2();
  const auto h2 = pow(h, 2);
  if (r2 > h2) {
    return 0;
  }
  return 315 / (64 * M_PI * pow(h, 9)) * pow(h2 - r2, 3);
}

// https://www.wolframalpha.com/input/?i=gradient+15%2F%28PI*h%5E6%29*%28h-x%29%5E3
inline __host__ __device__ REAL W_spiky_gradient_cuda(const Vector3R &r_vec, REAL h) {
  const auto r = r_vec.norm();
  if (r > h) {
    return 0;
  }
  return -45 / (M_PI * pow(h, 6)) * pow(h - r, 2);
}

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
  constexpr int default_capacity = 50;
  int init_search_result[] = {0, default_capacity};
  for (int i = 0; i < num_particle; i++) {
    cudaMalloc(&neighbor_search_results_host[i], sizeof(int)*default_capacity);
    cudaMemcpy(neighbor_search_results_host[i], init_search_result, sizeof(int)*2, cudaMemcpyHostToDevice);
  }
  cudaMemcpy(neighbor_search_results_dev, neighbor_search_results_host, sizeof(int *) * num_particle, cudaMemcpyHostToDevice);
}

void Fluid_cuda::simulate(REAL delta_t,
  const FluidParameters *cp,
  vector<CollisionObject *> *collision_objects) {
}

#endif