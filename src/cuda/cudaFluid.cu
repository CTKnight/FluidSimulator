#include <cmath>
#include <cstdio>

#include "cudaFluid.cuh"
#include "../kernel.h"

#ifdef BUILD_CUDA

__global__ void simulate_update_position_predict_position(
  int n,
  Vector3R *particle_positions, 
  Vector3R *particle_preditced_position, 
  Vector3R *particle_velocities, 
  const Vector3R external_accelerations, 
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
  }
}

__global__ void calculate_lambda(
  int n,
  Vector3R *particle_positions,
  int *neighbor_search_results,
  int *neighbor_search_results_size_prefix_sum,
  REAL particle_mass,
  REAL density,
  REAL epsilon,
  REAL h,
  REAL *lambda
) {
  for (int i = blockIdx.x * blockDim.x + threadIdx.x; i < n; i += blockDim.x * gridDim.x) {
    const auto &p_i = particle_positions[i];
    // line 10: calculate lambda
    const int neighbors_size_last = i == 0 ? 0 : neighbor_search_results_size_prefix_sum[i-1];
    const int neighbors_size_i = neighbor_search_results_size_prefix_sum[i]-neighbors_size_last;
    const int *neighbors_i = &neighbor_search_results[neighbors_size_last];
    // Eq 2
    REAL rho_i = 0;
    for (int jj = 0; jj <  neighbors_size_i; jj++) {
      int j = neighbors_i[jj];
      const auto &p_j = particle_positions[j];
      rho_i += W_poly6(p_i-p_j, h);
    }
    // add itself
    rho_i += W_poly6(p_i-p_i, h);
    rho_i *= particle_mass;
    // Eq 1
    const REAL C_i = rho_i / density - 1.;
    REAL C_i_p_k_2_sum = 0;
    // Eq 8
    // if k = j
    REAL C_i_p_k_j_2 = 0;
    // if k = i
    Vector3R C_i_p_k_i;
    for (int jj = 0; jj < neighbors_size_i; jj++) {
      int j = neighbors_i[jj];
      const auto &p_j = particle_positions[j];
      const auto W_spiky_gradient_i_j = W_spiky_gradient(p_i-p_j, h) * (p_i-p_j);
      C_i_p_k_i += W_spiky_gradient_i_j;
      C_i_p_k_j_2 += W_spiky_gradient_i_j.norm2();
    }
    C_i_p_k_2_sum += C_i_p_k_i.norm2();
    C_i_p_k_2_sum /= pow(density, 2);
    lambda[i] = - C_i / (C_i_p_k_2_sum+epsilon);
  }
}

__global__ void calculate_delta_pi_and_collision_response(
  int num_particles,
  Vector3R *particle_positions,
  int *neighbor_search_results,
  int *neighbor_search_results_size_prefix_sum,
  Vector3R *delta_p,
  REAL n,
  REAL k,
  REAL h,
  REAL density,
  REAL *lambda,
  int nObjs,
  Plane_cuda *collision_objects
) {
  for (int i = blockIdx.x * blockDim.x + threadIdx.x; i < num_particles; i += blockDim.x * gridDim.x) {
    const auto &p_i = particle_positions[i];
    // line 13: calculate delta p_i
    const int neighbors_size_last = i == 0 ? 0 : neighbor_search_results_size_prefix_sum[i-1];
    const int neighbors_size_i = neighbor_search_results_size_prefix_sum[i]-neighbors_size_last;
    const int *neighbors_i = &neighbor_search_results[neighbors_size_last];
    delta_p[i] = 0;
    const auto lambda_i = lambda[i];
    // Eq 12
    for (int jj = 0; jj <  neighbors_size_i; jj++) {
      int j = neighbors_i[jj];
      const auto &p_j = particle_positions[j];
      // Eq 13
      double s_corr = -k*pow(W_poly6(p_i-p_j, h)/W_poly6(0.3*h, h), n);
      delta_p[i] += (lambda_i+lambda[j]+s_corr) * W_spiky_gradient(p_i-p_j, h) 
        * (p_i-p_j);
    }
    delta_p[i] /= density;
    // line 14: collision detection and response
    // TODO: apply them
    for (int j = 0; j < nObjs; j++) {
      collision_objects[j].collide(particle_positions[i],delta_p[i]);
    }
  }
}

__global__ void update_predicted_positions(
  int num_particles,
  Vector3R *particle_predicted_positions,
  Vector3R *delta_p
) {
  for (int i = blockIdx.x * blockDim.x + threadIdx.x; i < num_particles; i += blockDim.x * gridDim.x) {
    particle_predicted_positions[i] += delta_p[i];
  }
}

__global__ void update_velocities(
  int num_particles,
  Vector3R *particle_positions,
  Vector3R *predicted_positions,
  Vector3R *particle_velocities,
  REAL delta_t
) {
  for (int i = blockIdx.x * blockDim.x + threadIdx.x; i < num_particles; i += blockDim.x * gridDim.x) {
    particle_velocities[i] = (predicted_positions[i] - particle_positions[i]) / delta_t;
  }
}

__global__ void apply_XSPH_viscosity(
  int num_particles,
  Vector3R *particle_positions,
  Vector3R *particle_velocities,
  int *neighbor_search_results,
  int *neighbor_search_results_size_prefix_sum,
  REAL particle_mass, REAL h, REAL c, REAL delta_t
) {
  for (int i = blockIdx.x * blockDim.x + threadIdx.x; i < num_particles; i += blockDim.x * gridDim.x) {
    const auto &p_i = particle_positions[i];
    const int neighbors_size_last = i == 0 ? 0 : neighbor_search_results_size_prefix_sum[i-1];
    const int neighbors_size_i = neighbor_search_results_size_prefix_sum[i]-neighbors_size_last;
    const int *neighbors_i = &neighbor_search_results[neighbors_size_last];
    // line 22: vorticity confinement and XSPH viscosity
    Vector3R f_vorticity;
    Vector3R omega_i;
    // Eq 17:
    Vector3R V_xsph;
    for (int jj = 0; jj <  neighbors_size_i; jj++) {
      int j = neighbors_i[jj];
      const auto &p_j = particle_positions[j];
      const auto &p_ij = p_i-p_j;
      const auto &v_ij = particle_velocities[j] - particle_velocities[i];
      // the smallest |p_ij| with h=0.1 gives >100 so c has to correct it to ~1
      V_xsph  += v_ij * W_poly6(p_ij, h);
      omega_i += cross(v_ij, W_spiky_gradient(p_ij, h)*p_ij);
    }
    // TODO: vorticity
    // const auto &eta = ;
    // const auto &N = eta.unit();
    // f_vorticity = epsilon*cross(N, omega_i);
    V_xsph *= c;
    particle_velocities[i] += V_xsph + f_vorticity / particle_mass * delta_t;
  }
}

void copy_predicted_positions_to_position(
  REAL3 *particle_position,
  REAL3 *particle_preditced_position, 
  size_t N
) {
  cudaMemcpy(particle_position, particle_preditced_position, N, cudaMemcpyDeviceToDevice);
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
  cudaFree(particle_predicted_positions_device);
  cudaFree(delta_p_device);
  cudaFree(lambda_device);
  cudaFree(neighbor_search_results_dev);
  cudaFree(neighbor_search_results_size_prefix_sum_dev);
}

void Fluid_cuda::init() {
  const auto num_particles = particle_positions->size();
  const auto SIZE_REAL3_N = sizeof(REAL3) * num_particles;

  cudaMalloc(&particle_positions_device, SIZE_REAL3_N);
  cudaMemcpy(particle_positions_device, particle_positions->data(), SIZE_REAL3_N, cudaMemcpyHostToDevice);

  cudaMalloc(&particle_velocities_device, SIZE_REAL3_N);
  cudaMemset(particle_velocities_device, 0, SIZE_REAL3_N);

  cudaMalloc(&particle_predicted_positions_device, SIZE_REAL3_N);
  cudaMalloc(&delta_p_device, SIZE_REAL3_N);
  cudaMalloc(&lambda_device, sizeof(REAL)*num_particles);

  neighbor_search_results_dev_capacity = num_particles * default_capacity;
  cudaMalloc(&neighbor_search_results_dev, sizeof(int) * neighbor_search_results_dev_capacity);
  cudaMalloc(&neighbor_search_results_size_prefix_sum_dev, sizeof(int) * num_particles);

  neighbor_search_results_host.resize(neighbor_search_results_dev_capacity);
  neighbor_search_results_size_prefix_sum_host.resize(num_particles);

  cudaDeviceSynchronize();
  nsearch.add_point_set(
    this->particle_positions->front().data(), 
    this->particle_positions->size(), true, true
  );
  nsearch.find_neighbors();
}

void Fluid_cuda::find_neighbors(){
  int num_particles = particle_positions->size();
  nsearch.find_neighbors();

  // serial calculate prefix_sum
  for (int i = 0; i < num_particles; i++) {
    auto &pointSet = nsearch.point_set(0);
    auto count = pointSet.n_neighbors(0, i);
    const auto last_sum = i == 0 ? 0 :  neighbor_search_results_size_prefix_sum_host[i-1];
    neighbor_search_results_size_prefix_sum_host[i] = last_sum + count;
    // range for result_i: [sum_{i-1}, sum_{i})
  }

  // ensure capacity
  const auto minCapacity = neighbor_search_results_size_prefix_sum_host.back();
  if (minCapacity > neighbor_search_results_dev_capacity) {
    neighbor_search_results_dev_capacity = minCapacity * 1.2;
    neighbor_search_results_host.resize(neighbor_search_results_dev_capacity);
    cudaFree(neighbor_search_results_dev);
    cudaMalloc(&neighbor_search_results_dev, sizeof(int) * neighbor_search_results_dev_capacity);
  }

  for (int i = 0; i < num_particles; i++) {
    // line 6: find neighboring particles
    auto &pointSet = nsearch.point_set(0);
    auto count = pointSet.n_neighbors(0, i);
    const int start = i == 0 ? 0 :  neighbor_search_results_size_prefix_sum_host[i-1];

    memcpy(&neighbor_search_results_host[start], pointSet.neighbor_list(0, i), sizeof(int)*count);
  }

  // copy neighbor results to device
  cudaMemcpy(
    neighbor_search_results_dev, 
    neighbor_search_results_host.data(), 
    sizeof(int) * neighbor_search_results_dev_capacity, 
    cudaMemcpyHostToDevice
  );
  // update size prefix sum to device
  cudaMemcpy(
    neighbor_search_results_size_prefix_sum_dev, 
    neighbor_search_results_size_prefix_sum_host.data(), 
    sizeof(int) * num_particles, 
    cudaMemcpyHostToDevice
  );

  cudaDeviceSynchronize();
}

void Fluid_cuda::simulate(REAL delta_t,
  const FluidParameters *fp,
  thrust::device_vector<Plane_cuda> &collision_objects) {
  int num_particles = particle_positions->size();
  const auto particle_positions_dev = REAL3AsVector3R(particle_positions_device);
  const auto particle_predicted_positions = REAL3AsVector3R(particle_predicted_positions_device);
  const auto particle_velocities = REAL3AsVector3R(particle_velocities_device);
  const auto delta_p = REAL3AsVector3R(delta_p_device);
  const auto density = fp->density;
  const auto particle_mass = fp->particle_mass;
  const auto damping = fp->damping;
  const auto solverIterations = fp->solverIterations;
  const auto h = fp->h;
  const auto epsilon = fp->epsilon;
  const auto n = fp->n;
  const auto k = fp->k;
  const auto c = fp->c;
  const Vector3R &external_accelerations = fp->external_forces;

  simulate_update_position_predict_position<<<num_particles,1>>>(
    num_particles,
    particle_positions_dev, 
    particle_predicted_positions, 
    particle_velocities, 
    external_accelerations, delta_t
  );

  find_neighbors();

  for (int iter = 0; iter < solverIterations; iter++) {
    calculate_lambda<<<num_particles,1>>>(
      num_particles, 
      particle_positions_dev,
      neighbor_search_results_dev,
      neighbor_search_results_size_prefix_sum_dev,
      particle_mass, density, epsilon, h,
      lambda_device
    );

    calculate_delta_pi_and_collision_response<<<num_particles,1>>>(
      num_particles,
      particle_positions_dev,
      neighbor_search_results_dev,
      neighbor_search_results_size_prefix_sum_dev,
      delta_p,
      n, k, h, density,
      lambda_device,
      collision_objects.size(),
      thrust::raw_pointer_cast(collision_objects.data())
    );

    update_predicted_positions<<<num_particles,1>>>(
      num_particles, 
      particle_predicted_positions, 
      delta_p
    );
  }

  update_velocities<<<num_particles,1>>>(
    num_particles,
    particle_positions_dev,
    particle_predicted_positions,
    particle_velocities,
    delta_t
  );

  apply_XSPH_viscosity<<<num_particles,1>>>(
    num_particles,
    particle_positions_dev,
    particle_velocities,
    neighbor_search_results_dev,
    neighbor_search_results_size_prefix_sum_dev,
    particle_mass, h, c, delta_t
  );

  const auto SIZE_REAL3_N = sizeof(REAL3) * num_particles;
  copy_predicted_positions_to_position(particle_positions_device, particle_predicted_positions_device, SIZE_REAL3_N);
  // copy result back to host
  cudaMemcpy(particle_positions->data(), particle_positions_device, SIZE_REAL3_N, cudaMemcpyDeviceToHost);
}

#endif