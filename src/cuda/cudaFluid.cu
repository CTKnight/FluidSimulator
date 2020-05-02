#include "cudaFluid.cuh"
#include <cmath>
#include <cstdio>

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

__global__ void simulate_update_position_predict_position(
  int n,
  Vector3R *particle_positions, 
  Vector3R *particle_preditced_position, 
  Vector3R *particle_velocities, 
  const Vector3R &external_accelerations, 
  REAL delta_t
) {
  for (int i = blockIdx.x * blockDim.x + threadIdx.x; i < n; i += blockDim.x * gridDim.x) {
    auto &positions_i = particle_positions[i];
    auto &velocities_i = particle_velocities[i];
    auto &preditced_positions_i = particle_preditced_position[i];
    if (i == 0) {
      printf("Thread 0 position: (%lf, %lf, %lf), acc: (%lf, %lf, %lf), delta_t: %lf\n", 
        positions_i.x, positions_i.y, positions_i.z,
        external_accelerations.x, external_accelerations.y, external_accelerations.z,
        delta_t
      );
    }
    velocities_i += external_accelerations * delta_t;
    preditced_positions_i = positions_i + velocities_i * delta_t;
    positions_i = preditced_positions_i;
 }
 __syncthreads();
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
  cudaFree(particle_preditced_positions_device);
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
  cudaMemcpy(particle_positions_device, particle_positions->data(), SIZE_REAL3_N, cudaMemcpyHostToDevice);

  cudaMalloc(&particle_velocities_device, SIZE_REAL3_N);
  cudaMemset(particle_velocities_device, 0, SIZE_REAL3_N);

  cudaMalloc(&particle_preditced_positions_device, SIZE_REAL3_N);
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
  cudaDeviceSynchronize();
}

void Fluid_cuda::simulate(REAL delta_t,
  const FluidParameters *fp,
  vector<CollisionObject *> *collision_objects) {
  int num_particles = particle_positions->size();
  const auto particle_positions_dev = REAL3AsVector3R(particle_positions_device);
  const auto particle_preditced_positions = REAL3AsVector3R(particle_preditced_positions_device);
  const auto particle_velocities = REAL3AsVector3R(particle_velocities_device);
  simulate_update_position_predict_position<<<num_particles,1>>>(
    num_particles,
    particle_positions_dev, 
    particle_preditced_positions, 
    particle_velocities, 
    fp->external_forces, delta_t
  );

  cudaDeviceSynchronize();

  const auto SIZE_REAL3_N = sizeof(REAL3) * num_particles;
  // copy result back to host
  cudaMemcpy(particle_positions->data(), particle_positions_device, SIZE_REAL3_N, cudaMemcpyDeviceToHost);
  cudaDeviceSynchronize();
}

#endif