#include "fluid/cuda.h"

#include <cuda_runtime.h>
#include <thrust/device_vector.h>
#include <thrust/iterator/constant_iterator.h>
#include <thrust/iterator/zip_iterator.h>
#include <thrust/reduce.h>
#include <thrust/scan.h>
#include <thrust/sort.h>
#include <thrust/tuple.h>

#include <cmath>
#include <cstdint>

namespace fluid {
namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr int kCellBits = 21;
constexpr int kCellBias = 1 << 20;
constexpr std::uint64_t kCellMask = (1ull << kCellBits) - 1ull;

__host__ __device__ inline std::uint64_t pack_cell(int x, int y, int z) {
  const std::uint64_t ux =
      static_cast<std::uint64_t>(x + kCellBias) & kCellMask;
  const std::uint64_t uy =
      static_cast<std::uint64_t>(y + kCellBias) & kCellMask;
  const std::uint64_t uz =
      static_cast<std::uint64_t>(z + kCellBias) & kCellMask;
  return (ux << (2 * kCellBits)) | (uy << kCellBits) | uz;
}

__host__ __device__ inline int cell_coord(float v, float cell) {
  return static_cast<int>(floorf(v / cell));
}

__device__ inline int find_cell(std::uint64_t key,
                                const std::uint64_t* keys,
                                int count) {
  int lo = 0;
  int hi = count - 1;
  while (lo <= hi) {
    const int mid = (lo + hi) >> 1;
    const std::uint64_t v = keys[mid];
    if (v == key) {
      return mid;
    }
    if (v < key) {
      lo = mid + 1;
    } else {
      hi = mid - 1;
    }
  }
  return -1;
}

__host__ __device__ inline float poly6_kernel(float r2, float h) {
  const float h2 = h * h;
  if (r2 > h2) {
    return 0.0f;
  }
  const float diff = h2 - r2;
  const float diff3 = diff * diff * diff;
  const float h4 = h2 * h2;
  const float h9 = h4 * h4 * h;
  const float coeff = 315.0f / (64.0f * kPi * h9);
  return coeff * diff3;
}

__host__ __device__ inline float spiky_gradient_factor(float r, float h) {
  if (r > h) {
    return 0.0f;
  }
  const float h2 = h * h;
  const float h6 = h2 * h2 * h2;
  const float diff = h - r;
  const float coeff = -45.0f / (kPi * h6);
  return coeff * diff * diff;
}

__host__ __device__ inline float pow_ratio_n(float ratio, int n) {
  if (n == 2) {
    return ratio * ratio;
  }
  if (n == 3) {
    return ratio * ratio * ratio;
  }
  if (n == 4) {
    const float r2 = ratio * ratio;
    return r2 * r2;
  }
  return powf(ratio, static_cast<float>(n));
}

struct CellKeyIndexLess {
  __host__ __device__ bool operator()(
      const thrust::tuple<std::uint64_t, int>& a,
      const thrust::tuple<std::uint64_t, int>& b) const {
    const std::uint64_t ka = thrust::get<0>(a);
    const std::uint64_t kb = thrust::get<0>(b);
    if (ka < kb) {
      return true;
    }
    if (kb < ka) {
      return false;
    }
    return thrust::get<1>(a) < thrust::get<1>(b);
  }
};

__global__ void predict_positions(int n,
                                  float* pos_x,
                                  float* pos_y,
                                  float* pos_z,
                                  float* vel_x,
                                  float* vel_y,
                                  float* vel_z,
                                  float fx,
                                  float fy,
                                  float fz,
                                  float dt,
                                  float* pred_x,
                                  float* pred_y,
                                  float* pred_z) {
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n) {
    return;
  }
  vel_x[i] += fx * dt;
  vel_y[i] += fy * dt;
  vel_z[i] += fz * dt;
  pred_x[i] = pos_x[i] + vel_x[i] * dt;
  pred_y[i] = pos_y[i] + vel_y[i] * dt;
  pred_z[i] = pos_z[i] + vel_z[i] * dt;
}

__global__ void compute_cell_keys(int n,
                                  const float* pred_x,
                                  const float* pred_y,
                                  const float* pred_z,
                                  float cell_size,
                                  std::uint64_t* keys,
                                  int* indices) {
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n) {
    return;
  }
  indices[i] = i;
  const int cx = cell_coord(pred_x[i], cell_size);
  const int cy = cell_coord(pred_y[i], cell_size);
  const int cz = cell_coord(pred_z[i], cell_size);
  keys[i] = pack_cell(cx, cy, cz);
}

__global__ void count_neighbors(int n,
                                const float* pred_x,
                                const float* pred_y,
                                const float* pred_z,
                                float cell_size,
                                float h2,
                                const std::uint64_t* cell_keys,
                                const int* cell_starts,
                                const int* cell_counts,
                                int cell_count,
                                const int* sorted_indices,
                                int* neighbor_counts) {
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n) {
    return;
  }
  const float px = pred_x[i];
  const float py = pred_y[i];
  const float pz = pred_z[i];
  const int cx = cell_coord(px, cell_size);
  const int cy = cell_coord(py, cell_size);
  const int cz = cell_coord(pz, cell_size);
  int count = 0;
  for (int dz = -1; dz <= 1; ++dz) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        const std::uint64_t key = pack_cell(cx + dx, cy + dy, cz + dz);
        const int cell_index = find_cell(key, cell_keys, cell_count);
        if (cell_index < 0) {
          continue;
        }
        const int start = cell_starts[cell_index];
        const int end = start + cell_counts[cell_index];
        for (int idx = start; idx < end; ++idx) {
          const int j = sorted_indices[idx];
          if (j == i) {
            continue;
          }
          const float dxp = px - pred_x[j];
          const float dyp = py - pred_y[j];
          const float dzp = pz - pred_z[j];
          const float r2 = dxp * dxp + dyp * dyp + dzp * dzp;
          if (r2 < h2) {
            ++count;
          }
        }
      }
    }
  }
  neighbor_counts[i] = count;
}

__global__ void fill_neighbors(int n,
                               const float* pred_x,
                               const float* pred_y,
                               const float* pred_z,
                               float cell_size,
                               float h2,
                               const std::uint64_t* cell_keys,
                               const int* cell_starts,
                               const int* cell_counts,
                               int cell_count,
                               const int* sorted_indices,
                               const int* prefix_sum,
                               int* neighbor_indices) {
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n) {
    return;
  }
  const int start_offset = (i == 0) ? 0 : prefix_sum[i - 1];
  int offset = 0;
  const float px = pred_x[i];
  const float py = pred_y[i];
  const float pz = pred_z[i];
  const int cx = cell_coord(px, cell_size);
  const int cy = cell_coord(py, cell_size);
  const int cz = cell_coord(pz, cell_size);
  for (int dz = -1; dz <= 1; ++dz) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        const std::uint64_t key = pack_cell(cx + dx, cy + dy, cz + dz);
        const int cell_index = find_cell(key, cell_keys, cell_count);
        if (cell_index < 0) {
          continue;
        }
        const int start = cell_starts[cell_index];
        const int end = start + cell_counts[cell_index];
        for (int idx = start; idx < end; ++idx) {
          const int j = sorted_indices[idx];
          if (j == i) {
            continue;
          }
          const float dxp = px - pred_x[j];
          const float dyp = py - pred_y[j];
          const float dzp = pz - pred_z[j];
          const float r2 = dxp * dxp + dyp * dyp + dzp * dzp;
          if (r2 < h2) {
            neighbor_indices[start_offset + offset] = j;
            ++offset;
          }
        }
      }
    }
  }
}

__global__ void compute_lambda(int n,
                               const float* pred_x,
                               const float* pred_y,
                               const float* pred_z,
                               const int* neighbor_indices,
                               const int* neighbor_prefix_sum,
                               float h,
                               float h2,
                               float min_r2,
                               float particle_mass,
                               float density,
                               float epsilon,
                               float* lambda,
                               float* rho_out) {
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n) {
    return;
  }
  const int start = (i == 0) ? 0 : neighbor_prefix_sum[i - 1];
  const int end = neighbor_prefix_sum[i];
  const float px = pred_x[i];
  const float py = pred_y[i];
  const float pz = pred_z[i];

  float rho = 0.0f;
  float grad_sum_x = 0.0f;
  float grad_sum_y = 0.0f;
  float grad_sum_z = 0.0f;
  float sum_grad2 = 0.0f;

  for (int idx = start; idx < end; ++idx) {
    const int j = neighbor_indices[idx];
    const float dx = px - pred_x[j];
    const float dy = py - pred_y[j];
    const float dz = pz - pred_z[j];
    const float r2 = dx * dx + dy * dy + dz * dz;
    rho += poly6_kernel(r2, h);

    if (r2 < h2) {
      const float r = sqrtf(r2 < min_r2 ? min_r2 : r2);
      const float grad_factor = spiky_gradient_factor(r, h);
      const float gx = grad_factor * dx;
      const float gy = grad_factor * dy;
      const float gz = grad_factor * dz;
      grad_sum_x += gx;
      grad_sum_y += gy;
      grad_sum_z += gz;
    }
  }

  rho += poly6_kernel(0.0f, h);
  rho *= particle_mass;
  rho_out[i] = rho;

  const float C = rho / density - 1.0f;
  const float inv_density = 1.0f / density;
  sum_grad2 = (grad_sum_x * grad_sum_x + grad_sum_y * grad_sum_y +
               grad_sum_z * grad_sum_z) *
              inv_density * inv_density;
  lambda[i] = -C / (sum_grad2 + epsilon);
}

__global__ void compute_delta(int n,
                              const float* pred_x,
                              const float* pred_y,
                              const float* pred_z,
                              const int* neighbor_indices,
                              const int* neighbor_prefix_sum,
                              float h,
                              float h2,
                              float min_r2,
                              float inv_density,
                              const float* lambda,
                              bool scorr_enabled,
                              float scorr_k,
                              int scorr_n,
                              float scorr_inv_wdq,
                              int plane_count,
                              const float* plane_nx,
                              const float* plane_ny,
                              const float* plane_nz,
                              const float* plane_d,
                              float particle_radius,
                              float* delta_x,
                              float* delta_y,
                              float* delta_z) {
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n) {
    return;
  }
  const int start = (i == 0) ? 0 : neighbor_prefix_sum[i - 1];
  const int end = neighbor_prefix_sum[i];
  const float px = pred_x[i];
  const float py = pred_y[i];
  const float pz = pred_z[i];
  const float lambda_i = lambda[i];

  float dx_sum = 0.0f;
  float dy_sum = 0.0f;
  float dz_sum = 0.0f;

  for (int idx = start; idx < end; ++idx) {
    const int j = neighbor_indices[idx];
    const float dx = px - pred_x[j];
    const float dy = py - pred_y[j];
    const float dz = pz - pred_z[j];
    const float r2 = dx * dx + dy * dy + dz * dz;
    if (r2 < h2) {
      const float r = sqrtf(r2 < min_r2 ? min_r2 : r2);
      const float grad_factor = spiky_gradient_factor(r, h);
      float s = lambda_i + lambda[j];
      if (scorr_enabled && scorr_inv_wdq > 0.0f) {
        const float W = poly6_kernel(r2, h);
        const float ratio = W * scorr_inv_wdq;
        const float scorr = -scorr_k * pow_ratio_n(ratio, scorr_n);
        s += scorr;
      }
      dx_sum += s * grad_factor * dx;
      dy_sum += s * grad_factor * dy;
      dz_sum += s * grad_factor * dz;
    }
  }

  float dx_out = dx_sum * inv_density;
  float dy_out = dy_sum * inv_density;
  float dz_out = dz_sum * inv_density;

  if (plane_count > 0) {
    float px_pred = px + dx_out;
    float py_pred = py + dy_out;
    float pz_pred = pz + dz_out;
    for (int p = 0; p < plane_count; ++p) {
      const float sd =
          plane_nx[p] * px_pred + plane_ny[p] * py_pred + plane_nz[p] * pz_pred -
          plane_d[p];
      const float penetration = -sd;
      if (penetration > 0.0f) {
        px_pred += plane_nx[p] * penetration;
        py_pred += plane_ny[p] * penetration;
        pz_pred += plane_nz[p] * penetration;
      }
    }
    dx_out = px_pred - px;
    dy_out = py_pred - py;
    dz_out = pz_pred - pz;
  }

  delta_x[i] = dx_out;
  delta_y[i] = dy_out;
  delta_z[i] = dz_out;
}

__global__ void apply_delta(int n,
                            float* pred_x,
                            float* pred_y,
                            float* pred_z,
                            const float* delta_x,
                            const float* delta_y,
                            const float* delta_z) {
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n) {
    return;
  }
  pred_x[i] += delta_x[i];
  pred_y[i] += delta_y[i];
  pred_z[i] += delta_z[i];
}

__global__ void update_velocity(int n,
                                const float* pos_x,
                                const float* pos_y,
                                const float* pos_z,
                                const float* pred_x,
                                const float* pred_y,
                                const float* pred_z,
                                float* vel_x,
                                float* vel_y,
                                float* vel_z,
                                float dt) {
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n) {
    return;
  }
  vel_x[i] = (pred_x[i] - pos_x[i]) / dt;
  vel_y[i] = (pred_y[i] - pos_y[i]) / dt;
  vel_z[i] = (pred_z[i] - pos_z[i]) / dt;
}

__global__ void commit_positions(int n,
                                 float* pos_x,
                                 float* pos_y,
                                 float* pos_z,
                                 const float* pred_x,
                                 const float* pred_y,
                                 const float* pred_z) {
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n) {
    return;
  }
  pos_x[i] = pred_x[i];
  pos_y[i] = pred_y[i];
  pos_z[i] = pred_z[i];
}

__global__ void xsph_compute_dv(int n,
                                const float* pos_x,
                                const float* pos_y,
                                const float* pos_z,
                                const float* vel_x,
                                const float* vel_y,
                                const float* vel_z,
                                const float* rho,
                                const int* neighbor_indices,
                                const int* neighbor_prefix_sum,
                                float h,
                                float h2,
                                float particle_mass,
                                float* dv_x,
                                float* dv_y,
                                float* dv_z) {
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n) {
    return;
  }
  const int start = (i == 0) ? 0 : neighbor_prefix_sum[i - 1];
  const int end = neighbor_prefix_sum[i];
  const float vx = vel_x[i];
  const float vy = vel_y[i];
  const float vz = vel_z[i];
  float dvx = 0.0f;
  float dvy = 0.0f;
  float dvz = 0.0f;
  const float px = pos_x[i];
  const float py = pos_y[i];
  const float pz = pos_z[i];
  for (int idx = start; idx < end; ++idx) {
    const int j = neighbor_indices[idx];
    const float dx = px - pos_x[j];
    const float dy = py - pos_y[j];
    const float dz = pz - pos_z[j];
    const float r2 = dx * dx + dy * dy + dz * dz;
    if (r2 < h2) {
      const float W = poly6_kernel(r2, h);
      const float inv_rho_j = (rho[j] > 0.0f) ? (particle_mass / rho[j]) : 0.0f;
      dvx += (vel_x[j] - vx) * W * inv_rho_j;
      dvy += (vel_y[j] - vy) * W * inv_rho_j;
      dvz += (vel_z[j] - vz) * W * inv_rho_j;
    }
  }
  dv_x[i] = dvx;
  dv_y[i] = dvy;
  dv_z[i] = dvz;
}

__global__ void xsph_apply_dv(int n,
                              float* vel_x,
                              float* vel_y,
                              float* vel_z,
                              const float* dv_x,
                              const float* dv_y,
                              const float* dv_z,
                              float visc_c) {
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n) {
    return;
  }
  vel_x[i] += visc_c * dv_x[i];
  vel_y[i] += visc_c * dv_y[i];
  vel_z[i] += visc_c * dv_z[i];
}

}  // namespace

int cuda_version() {
  return 1;
}

void cuda_step(const Params& params, State& state) {
  const std::size_t count = state.size();
  if (count == 0) {
    state.time += params.dt;
    return;
  }

  const float dt = params.dt;
  const float h = params.h;
  const float h2 = h * h;
  const float min_r = 0.01f * h;
  const float min_r2 = min_r * min_r;
  const float density = params.density;
  const float inv_density = 1.0f / density;
  const float particle_mass = params.particle_mass;
  const int solver_iterations = params.solver_iterations;

  const bool scorr_enabled = params.enable_scorr && params.scorr_k != 0.0f;
  float scorr_inv_wdq = 0.0f;
  if (scorr_enabled && h > 0.0f) {
    const float scorr_dq = params.scorr_dq_coeff * h;
    const float wdq = poly6_kernel(scorr_dq * scorr_dq, h);
    if (wdq > 1e-12f) {
      scorr_inv_wdq = 1.0f / wdq;
    }
  }

  thrust::device_vector<float> d_pos_x(state.pos_x.begin(), state.pos_x.end());
  thrust::device_vector<float> d_pos_y(state.pos_y.begin(), state.pos_y.end());
  thrust::device_vector<float> d_pos_z(state.pos_z.begin(), state.pos_z.end());
  thrust::device_vector<float> d_vel_x(state.vel_x.begin(), state.vel_x.end());
  thrust::device_vector<float> d_vel_y(state.vel_y.begin(), state.vel_y.end());
  thrust::device_vector<float> d_vel_z(state.vel_z.begin(), state.vel_z.end());

  thrust::device_vector<float> d_pred_x(count);
  thrust::device_vector<float> d_pred_y(count);
  thrust::device_vector<float> d_pred_z(count);
  thrust::device_vector<float> d_delta_x(count);
  thrust::device_vector<float> d_delta_y(count);
  thrust::device_vector<float> d_delta_z(count);
  thrust::device_vector<float> d_lambda(count);
  thrust::device_vector<float> d_rho(count);

  const int plane_count = static_cast<int>(params.planes.size());
  thrust::device_vector<float> d_plane_nx(params.planes.nx.begin(),
                                          params.planes.nx.end());
  thrust::device_vector<float> d_plane_ny(params.planes.ny.begin(),
                                          params.planes.ny.end());
  thrust::device_vector<float> d_plane_nz(params.planes.nz.begin(),
                                          params.planes.nz.end());
  thrust::device_vector<float> d_plane_d(params.planes.d.begin(),
                                         params.planes.d.end());

  const int threads = 256;
  const int blocks = static_cast<int>((count + threads - 1) / threads);

  predict_positions<<<blocks, threads>>>(
      static_cast<int>(count),
      thrust::raw_pointer_cast(d_pos_x.data()),
      thrust::raw_pointer_cast(d_pos_y.data()),
      thrust::raw_pointer_cast(d_pos_z.data()),
      thrust::raw_pointer_cast(d_vel_x.data()),
      thrust::raw_pointer_cast(d_vel_y.data()),
      thrust::raw_pointer_cast(d_vel_z.data()),
      params.external_forces.x,
      params.external_forces.y,
      params.external_forces.z,
      dt,
      thrust::raw_pointer_cast(d_pred_x.data()),
      thrust::raw_pointer_cast(d_pred_y.data()),
      thrust::raw_pointer_cast(d_pred_z.data()));

  thrust::device_vector<std::uint64_t> d_cell_keys(count);
  thrust::device_vector<int> d_particle_indices(count);
  compute_cell_keys<<<blocks, threads>>>(
      static_cast<int>(count),
      thrust::raw_pointer_cast(d_pred_x.data()),
      thrust::raw_pointer_cast(d_pred_y.data()),
      thrust::raw_pointer_cast(d_pred_z.data()),
      h,
      thrust::raw_pointer_cast(d_cell_keys.data()),
      thrust::raw_pointer_cast(d_particle_indices.data()));

  auto zip_begin = thrust::make_zip_iterator(
      thrust::make_tuple(d_cell_keys.begin(), d_particle_indices.begin()));
  auto zip_end = zip_begin + static_cast<long>(count);
  thrust::sort(zip_begin, zip_end, CellKeyIndexLess());

  thrust::device_vector<std::uint64_t> d_unique_keys(count);
  thrust::device_vector<int> d_cell_counts(count);
  auto reduce_end = thrust::reduce_by_key(
      d_cell_keys.begin(),
      d_cell_keys.end(),
      thrust::make_constant_iterator(1),
      d_unique_keys.begin(),
      d_cell_counts.begin());
  const int unique_count =
      static_cast<int>(reduce_end.first - d_unique_keys.begin());
  thrust::device_vector<int> d_cell_starts(unique_count);
  thrust::exclusive_scan(d_cell_counts.begin(),
                         d_cell_counts.begin() + unique_count,
                         d_cell_starts.begin());

  thrust::device_vector<int> d_neighbor_counts(count);
  count_neighbors<<<blocks, threads>>>(
      static_cast<int>(count),
      thrust::raw_pointer_cast(d_pred_x.data()),
      thrust::raw_pointer_cast(d_pred_y.data()),
      thrust::raw_pointer_cast(d_pred_z.data()),
      h,
      h2,
      thrust::raw_pointer_cast(d_unique_keys.data()),
      thrust::raw_pointer_cast(d_cell_starts.data()),
      thrust::raw_pointer_cast(d_cell_counts.data()),
      unique_count,
      thrust::raw_pointer_cast(d_particle_indices.data()),
      thrust::raw_pointer_cast(d_neighbor_counts.data()));

  thrust::device_vector<int> d_neighbor_prefix_sum(count);
  thrust::inclusive_scan(d_neighbor_counts.begin(),
                         d_neighbor_counts.end(),
                         d_neighbor_prefix_sum.begin());

  int total_neighbors = 0;
  if (count > 0) {
    cudaMemcpy(&total_neighbors,
               thrust::raw_pointer_cast(d_neighbor_prefix_sum.data()) +
                   (count - 1),
               sizeof(int),
               cudaMemcpyDeviceToHost);
  }
  thrust::device_vector<int> d_neighbor_indices(total_neighbors);
  fill_neighbors<<<blocks, threads>>>(
      static_cast<int>(count),
      thrust::raw_pointer_cast(d_pred_x.data()),
      thrust::raw_pointer_cast(d_pred_y.data()),
      thrust::raw_pointer_cast(d_pred_z.data()),
      h,
      h2,
      thrust::raw_pointer_cast(d_unique_keys.data()),
      thrust::raw_pointer_cast(d_cell_starts.data()),
      thrust::raw_pointer_cast(d_cell_counts.data()),
      unique_count,
      thrust::raw_pointer_cast(d_particle_indices.data()),
      thrust::raw_pointer_cast(d_neighbor_prefix_sum.data()),
      thrust::raw_pointer_cast(d_neighbor_indices.data()));

  for (int iter = 0; iter < solver_iterations; ++iter) {
    compute_lambda<<<blocks, threads>>>(
        static_cast<int>(count),
        thrust::raw_pointer_cast(d_pred_x.data()),
        thrust::raw_pointer_cast(d_pred_y.data()),
        thrust::raw_pointer_cast(d_pred_z.data()),
        thrust::raw_pointer_cast(d_neighbor_indices.data()),
        thrust::raw_pointer_cast(d_neighbor_prefix_sum.data()),
        h,
        h2,
        min_r2,
        particle_mass,
        density,
        params.epsilon,
        thrust::raw_pointer_cast(d_lambda.data()),
        thrust::raw_pointer_cast(d_rho.data()));

    compute_delta<<<blocks, threads>>>(
        static_cast<int>(count),
        thrust::raw_pointer_cast(d_pred_x.data()),
        thrust::raw_pointer_cast(d_pred_y.data()),
        thrust::raw_pointer_cast(d_pred_z.data()),
        thrust::raw_pointer_cast(d_neighbor_indices.data()),
        thrust::raw_pointer_cast(d_neighbor_prefix_sum.data()),
        h,
        h2,
        min_r2,
        inv_density,
        thrust::raw_pointer_cast(d_lambda.data()),
        scorr_enabled,
        params.scorr_k,
        params.scorr_n,
        scorr_inv_wdq,
        plane_count,
        thrust::raw_pointer_cast(d_plane_nx.data()),
        thrust::raw_pointer_cast(d_plane_ny.data()),
        thrust::raw_pointer_cast(d_plane_nz.data()),
        thrust::raw_pointer_cast(d_plane_d.data()),
        params.particle_radius,
        thrust::raw_pointer_cast(d_delta_x.data()),
        thrust::raw_pointer_cast(d_delta_y.data()),
        thrust::raw_pointer_cast(d_delta_z.data()));

    apply_delta<<<blocks, threads>>>(
        static_cast<int>(count),
        thrust::raw_pointer_cast(d_pred_x.data()),
        thrust::raw_pointer_cast(d_pred_y.data()),
        thrust::raw_pointer_cast(d_pred_z.data()),
        thrust::raw_pointer_cast(d_delta_x.data()),
        thrust::raw_pointer_cast(d_delta_y.data()),
        thrust::raw_pointer_cast(d_delta_z.data()));
  }

  update_velocity<<<blocks, threads>>>(
      static_cast<int>(count),
      thrust::raw_pointer_cast(d_pos_x.data()),
      thrust::raw_pointer_cast(d_pos_y.data()),
      thrust::raw_pointer_cast(d_pos_z.data()),
      thrust::raw_pointer_cast(d_pred_x.data()),
      thrust::raw_pointer_cast(d_pred_y.data()),
      thrust::raw_pointer_cast(d_pred_z.data()),
      thrust::raw_pointer_cast(d_vel_x.data()),
      thrust::raw_pointer_cast(d_vel_y.data()),
      thrust::raw_pointer_cast(d_vel_z.data()),
      dt);

  commit_positions<<<blocks, threads>>>(
      static_cast<int>(count),
      thrust::raw_pointer_cast(d_pos_x.data()),
      thrust::raw_pointer_cast(d_pos_y.data()),
      thrust::raw_pointer_cast(d_pos_z.data()),
      thrust::raw_pointer_cast(d_pred_x.data()),
      thrust::raw_pointer_cast(d_pred_y.data()),
      thrust::raw_pointer_cast(d_pred_z.data()));

  if (params.enable_xsph && params.visc_c != 0.0f) {
    thrust::device_vector<float> d_dv_x(count);
    thrust::device_vector<float> d_dv_y(count);
    thrust::device_vector<float> d_dv_z(count);
    xsph_compute_dv<<<blocks, threads>>>(
        static_cast<int>(count),
        thrust::raw_pointer_cast(d_pos_x.data()),
        thrust::raw_pointer_cast(d_pos_y.data()),
        thrust::raw_pointer_cast(d_pos_z.data()),
        thrust::raw_pointer_cast(d_vel_x.data()),
        thrust::raw_pointer_cast(d_vel_y.data()),
        thrust::raw_pointer_cast(d_vel_z.data()),
        thrust::raw_pointer_cast(d_rho.data()),
        thrust::raw_pointer_cast(d_neighbor_indices.data()),
        thrust::raw_pointer_cast(d_neighbor_prefix_sum.data()),
        h,
        h2,
        particle_mass,
        thrust::raw_pointer_cast(d_dv_x.data()),
        thrust::raw_pointer_cast(d_dv_y.data()),
        thrust::raw_pointer_cast(d_dv_z.data()));
    xsph_apply_dv<<<blocks, threads>>>(
        static_cast<int>(count),
        thrust::raw_pointer_cast(d_vel_x.data()),
        thrust::raw_pointer_cast(d_vel_y.data()),
        thrust::raw_pointer_cast(d_vel_z.data()),
        thrust::raw_pointer_cast(d_dv_x.data()),
        thrust::raw_pointer_cast(d_dv_y.data()),
        thrust::raw_pointer_cast(d_dv_z.data()),
        params.visc_c);
  }

  thrust::copy(d_pos_x.begin(), d_pos_x.end(), state.pos_x.begin());
  thrust::copy(d_pos_y.begin(), d_pos_y.end(), state.pos_y.begin());
  thrust::copy(d_pos_z.begin(), d_pos_z.end(), state.pos_z.begin());
  thrust::copy(d_vel_x.begin(), d_vel_x.end(), state.vel_x.begin());
  thrust::copy(d_vel_y.begin(), d_vel_y.end(), state.vel_y.begin());
  thrust::copy(d_vel_z.begin(), d_vel_z.end(), state.vel_z.begin());
  state.time += dt;
}

}  // namespace fluid
