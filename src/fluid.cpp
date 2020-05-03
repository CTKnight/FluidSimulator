#include <math.h>
#include <random>
#include <vector>
#include <exception>
#include <omp.h>

#include "fluid.h"
#include "collision/plane.h"
#include "collision/sphere.h"
#include "kernel.h"

using namespace std;

Fluid::Fluid(
    unique_ptr<vector<REAL3>> &&particle_positions,
    unique_ptr<vector<REAL3>> &&particle_velocities,
    REAL h
): nsearch(h) {
  if (particle_positions == nullptr) {
    throw std::runtime_error("particle_positions == nullptr!");
  }
  this->particle_positions = std::move(particle_positions);
}

void Fluid::init() {
  const auto num_particle = particle_positions->size();
  if (particle_velocities != nullptr) {
    if (num_particle != particle_velocities->size()) {
      throw std::runtime_error("particle_positions->size()  != particle_velocities->size()!");
    }
    this->particle_velocities = std::move(particle_velocities);
  } else {
    auto velocities = make_unique<vector<REAL3>>();
    velocities->resize(this->particle_positions->size());
    // Init to 0
    memset(velocities->data(), 0, sizeof(REAL3)*num_particle);
    this->particle_velocities = std::move(velocities);
  }
  this->particle_predicted_positions.resize(num_particle);
  this->delta_p.resize(num_particle);
  this->lambda.resize(num_particle);

  // Init nsearch
  nsearch.add_point_set(
    this->particle_positions->front().data(), 
    this->particle_positions->size(), true, true
  );
  nsearch.find_neighbors();
	neighbor_search_results.resize(num_particle);
}

void Fluid::simulate(
  REAL frames_per_sec, REAL simulation_steps, 
  const std::shared_ptr<FluidParameters> &cp, vector<CollisionObject *> *collision_objects
) {
  double delta_t = 1.0f / frames_per_sec / simulation_steps;
  auto &particle_positions = REAL3AsVector3R(*this->particle_positions);
  const auto num_particle = particle_positions.size();
  auto &particle_velocities = REAL3AsVector3R(*this->particle_velocities);
  auto &predicted_positions = REAL3AsVector3R(this->particle_predicted_positions);
  auto &delta_p = REAL3AsVector3R(this->delta_p);
  const auto density = cp->density;
  const auto particle_mass = cp->particle_mass;
  const auto damping = cp->damping;
  const auto solverIterations = cp->solverIterations;
  const auto h = cp->h;
  const auto epsilon = cp->epsilon;
  const auto n = cp->n;
  const auto k = cp->k;
  const auto c = cp->c;
  const Vector3R &external_accelerations = cp->external_forces;

  #pragma omp parallel for
  for (int i = 0; i < num_particle; i++) {
    // line 2: apply forces
    particle_velocities[i] += external_accelerations * delta_t;
    // line 3: predict positions
    predicted_positions[i] = particle_positions[i] + particle_velocities[i] * delta_t;
  }

  // update nsearch
  nsearch.find_neighbors();
  #pragma omp parallel for
  for (int i = 0; i < num_particle; i++) {
    // line 6: find neighboring particles
    neighbor_search_results[i].clear();
    #ifdef BUILD_CUDA
    auto &pointSet = nsearch.point_set(0);
    auto count = pointSet.n_neighbors(0, i);
    neighbor_search_results[i].resize(1);
    // TODO: refactor as cudaFluid.cu implemetation
    for (int j = 0; j < count; j++) {
      neighbor_search_results[i][0].emplace_back(pointSet.neighbor(0, i, j));
    }
    #else
    nsearch.find_neighbors(0, i, neighbor_search_results[i]);
    #endif
  }

  for (int iter = 0; iter < solverIterations; iter++) {
    #pragma omp parallel for
    for (int i = 0; i < num_particle; i++) {
      const auto &p_i = particle_positions[i];
      // line 10: calculate lambda
      const auto &neighbors = neighbor_search_results[i][0];
      // Eq 2
      double rho_i = 0;
      for (const auto &j: neighbors) {
        const auto &p_j = particle_positions[j];
        rho_i += W_poly6(p_i-p_j, h);
      }
      // add itself
      rho_i += W_poly6(p_i-p_i, h);
      rho_i *= particle_mass;
      // Eq 1
      const auto C_i = rho_i / density - 1.;
      double C_i_p_k_2_sum = 0;
      // Eq 8
      // if k = j
      double C_i_p_k_j_2 = 0;
      // if k = i
      Vector3R C_i_p_k_i;
      for (const auto &j: neighbors) {
        const auto &p_j = particle_positions[j];
        const auto W_spiky_gradient_i_j = W_spiky_gradient(p_i-p_j, h) * (p_i-p_j);
        C_i_p_k_i += W_spiky_gradient_i_j;
        C_i_p_k_j_2 += W_spiky_gradient_i_j.norm2();
      }
      C_i_p_k_2_sum += C_i_p_k_i.norm2();
      C_i_p_k_2_sum /= pow(density, 2);
      lambda[i] = - C_i / (C_i_p_k_2_sum+epsilon);
    }

    #pragma omp parallel for
    for (int i = 0; i < num_particle; i++) {
      const auto &p_i = particle_positions[i];
      // line 13: calculate delta p_i
      const auto &neighbors = neighbor_search_results[i][0];
      delta_p[i] = 0;
      const auto lambda_i = lambda[i];
      // Eq 12
      for (const auto &j: neighbors) {
        const auto &p_j = particle_positions[j];
        // Eq 13
        double s_corr = -k*pow(W_poly6(p_i-p_j, h)/W_poly6(0.3*h, h), n);
        delta_p[i] += (lambda_i+lambda[j]+s_corr) * W_spiky_gradient(p_i-p_j, h) 
          * (p_i-p_j);
      }
      delta_p[i] /= density;
      // line 14: collision detection and response
      for (const auto co: *collision_objects) {
        co->collide(particle_positions[i],delta_p[i]);
      }
    }

    #pragma omp parallel for
    for (int i = 0; i < num_particle; i++) {
      // line 17: update position
      predicted_positions[i] += delta_p[i];
    }
  }

  #pragma omp parallel for
  for (int i = 0; i < num_particle; i++) {
    // line 21: update velocity
    particle_velocities[i] = (predicted_positions[i] - particle_positions[i]) / delta_t;
  }

  #pragma omp parallel for
  for (int i = 0; i < num_particle; i++) {
    const auto &p_i = particle_positions[i];
    const auto &neighbors = neighbor_search_results[i][0];

    // line 22: vorticity confinement and XSPH viscosity
    Vector3R f_vorticity;
    Vector3R omega_i;
    // Eq 17:
    Vector3R V_xsph;
    for (const auto &j: neighbors) {
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

  // line 23: update position
  memcpy(particle_positions.data(), predicted_positions.data(), sizeof(REAL3)*num_particle);
}

void Fluid::reset() {

}
