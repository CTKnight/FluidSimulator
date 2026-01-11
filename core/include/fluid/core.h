#pragma once

#include <cmath>
#include <cstddef>
#include <vector>

namespace fluid {

int core_version();

struct Params {
  float dt = 1.0f / 60.0f;
  enum class Backend {
    Cpu,
    Cuda,
  };
  Backend backend = Backend::Cpu;
  float density = 6000.0f;
  // calculated based on density and particle radius
  float particle_mass = 0.0f;
  // calculated based on density and particle radius
  float h = 0.0f;
  float particle_radius = 0.01f;
  float epsilon = 600.0f;
  int solver_iterations = 4;
  float neighbor_reserve_factor = 1.5f;
  bool use_uniform_grid = true;
  bool enable_scorr = true;
  bool enable_xsph = true;
  bool enable_vorticity = false;
  float scorr_k = 0.00005f;
  int scorr_n = 4;
  float scorr_dq_coeff = 0.3f;
  float visc_c = 0.0002f;
  float vort_epsilon = 0.5f;
  float vort_norm_eps = 1e-6f;
  struct Vec3 {
    float x = 0.0f;
    float y = -9.8f;
    float z = 0.0f;
  } external_forces;
  struct PlaneSoA {
    std::vector<float> nx;
    std::vector<float> ny;
    std::vector<float> nz;
    std::vector<float> d;

    std::size_t size() const { return nx.size(); }
    void resize(std::size_t count) {
      nx.resize(count);
      ny.resize(count);
      nz.resize(count);
      d.resize(count);
    }
    void clear() {
      nx.clear();
      ny.clear();
      nz.clear();
      d.clear();
    }
    void add(float nx_in, float ny_in, float nz_in, float d_in) {
      nx.push_back(nx_in);
      ny.push_back(ny_in);
      nz.push_back(nz_in);
      d.push_back(d_in);
    }
    void add_normalized(float nx_in, float ny_in, float nz_in, float d_in) {
      const float len_sq = nx_in * nx_in + ny_in * ny_in + nz_in * nz_in;
      if (len_sq > 0.0f) {
        const float inv_len = 1.0f / std::sqrt(len_sq);
        add(nx_in * inv_len, ny_in * inv_len, nz_in * inv_len, d_in);
      } else {
        add(nx_in, ny_in, nz_in, d_in);
      }
    }
  } planes;
};

struct CpuScratch {
  struct CellKey {
    int x = 0;
    int y = 0;
    int z = 0;
  };
  struct CellEntry {
    CellKey key;
    int particle = 0;
  };

  std::vector<float> pred_x;
  std::vector<float> pred_y;
  std::vector<float> pred_z;
  std::vector<float> delta_x;
  std::vector<float> delta_y;
  std::vector<float> delta_z;
  std::vector<float> lambda;
  std::vector<float> rho;
  std::vector<float> dv_x;
  std::vector<float> dv_y;
  std::vector<float> dv_z;
  std::vector<float> omega_x;
  std::vector<float> omega_y;
  std::vector<float> omega_z;
  std::vector<float> omega_mag;
  std::vector<float> eta_x;
  std::vector<float> eta_y;
  std::vector<float> eta_z;
  std::vector<int> neighbor_indices;
  std::vector<int> neighbor_prefix_sum;
  std::vector<CellEntry> grid_entries;
  std::vector<CellKey> grid_keys;
  std::vector<int> grid_starts;
  std::vector<int> grid_ends;

  void resize(std::size_t particle_count);
  void clear_neighbors();
};

struct State {
  std::vector<float> pos_x;
  std::vector<float> pos_y;
  std::vector<float> pos_z;
  std::vector<float> vel_x;
  std::vector<float> vel_y;
  std::vector<float> vel_z;
  CpuScratch cpu;
  float time = 0.0f;

  std::size_t size() const { return pos_x.size(); }
};

State make_state(std::size_t particle_count);
void step(const Params& params, State& state);

}  // namespace fluid
