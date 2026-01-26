#include "fluid/core.h"

#include <algorithm>
#include <cmath>

namespace fluid {

namespace {

constexpr float kPi = 3.14159265358979323846f;

bool cell_key_less(const CpuScratch::CellKey& a,
                   const CpuScratch::CellKey& b) {
  if (a.x != b.x) {
    return a.x < b.x;
  }
  if (a.y != b.y) {
    return a.y < b.y;
  }
  return a.z < b.z;
}

bool cell_key_equal(const CpuScratch::CellKey& a,
                    const CpuScratch::CellKey& b) {
  return a.x == b.x && a.y == b.y && a.z == b.z;
}

CpuScratch::CellKey cell_key_from_pos(float x, float y, float z, float cell) {
  const float inv = 1.0f / cell;
  return CpuScratch::CellKey{
      static_cast<int>(std::floor(x * inv)),
      static_cast<int>(std::floor(y * inv)),
      static_cast<int>(std::floor(z * inv))};
}
float poly6_kernel(float r2, float h) {
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

float spiky_gradient_factor(float r, float h) {
  if (r > h) {
    return 0.0f;
  }
  const float h2 = h * h;
  const float h6 = h2 * h2 * h2;
  const float diff = h - r;
  const float coeff = -45.0f / (kPi * h6);
  return coeff * diff * diff;
}

float pow_ratio_n(float ratio, int n) {
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
  return std::pow(ratio, static_cast<float>(n));
}

}  // namespace

int core_version() {
  return 1;
}

void CpuScratch::resize(std::size_t particle_count) {
  pred_x.resize(particle_count, 0.0f);
  pred_y.resize(particle_count, 0.0f);
  pred_z.resize(particle_count, 0.0f);
  delta_x.resize(particle_count, 0.0f);
  delta_y.resize(particle_count, 0.0f);
  delta_z.resize(particle_count, 0.0f);
  lambda.resize(particle_count, 0.0f);
  rho.resize(particle_count, 0.0f);
  dv_x.resize(particle_count, 0.0f);
  dv_y.resize(particle_count, 0.0f);
  dv_z.resize(particle_count, 0.0f);
  omega_x.resize(particle_count, 0.0f);
  omega_y.resize(particle_count, 0.0f);
  omega_z.resize(particle_count, 0.0f);
  omega_mag.resize(particle_count, 0.0f);
  eta_x.resize(particle_count, 0.0f);
  eta_y.resize(particle_count, 0.0f);
  eta_z.resize(particle_count, 0.0f);
  neighbor_prefix_sum.resize(particle_count, 0);
  grid_entries.resize(particle_count);
}

void CpuScratch::clear_neighbors() {
  neighbor_indices.clear();
  std::fill(neighbor_prefix_sum.begin(), neighbor_prefix_sum.end(), 0);
}

State make_state(std::size_t particle_count) {
  State state;
  state.pos_x.resize(particle_count, 0.0f);
  state.pos_y.resize(particle_count, 0.0f);
  state.pos_z.resize(particle_count, 0.0f);
  state.vel_x.resize(particle_count, 0.0f);
  state.vel_y.resize(particle_count, 0.0f);
  state.vel_z.resize(particle_count, 0.0f);
  state.cpu.resize(particle_count);
  return state;
}

void step(const Params& params, State& state) {
  const std::size_t count = state.size();
  state.cpu.resize(count);
  if (count == 0) {
    state.time += params.dt;
    return;
  }

  CpuScratch& scratch = state.cpu;
  const std::size_t previous_neighbors = scratch.neighbor_indices.size();
  scratch.clear_neighbors();
  if (previous_neighbors > 0 && params.neighbor_reserve_factor > 0.0f) {
    const std::size_t neighbor_reserve = static_cast<std::size_t>(
        std::ceil(previous_neighbors * params.neighbor_reserve_factor));
    if (neighbor_reserve > scratch.neighbor_indices.capacity()) {
      scratch.neighbor_indices.reserve(neighbor_reserve);
    }
  }

  const float dt = params.dt;
  const float h = params.h;
  const float h2 = h * h;
  const float min_r = 0.01f * h;
  const float min_r2 = min_r * min_r;
  const bool scorr_enabled = params.enable_scorr && params.scorr_k != 0.0f;
  const float scorr_dq_coeff =
      (params.scorr_dq_coeff > 0.0f) ? params.scorr_dq_coeff : 0.3f;
  const float scorr_dq = scorr_dq_coeff * h;
  const float wdq = scorr_enabled ? poly6_kernel(scorr_dq * scorr_dq, h) : 0.0f;
  const float scorr_inv_wdq = (wdq > 1e-12f) ? (1.0f / wdq) : 0.0f;

  // Apply forces and predict positions (PBF integration step).
#ifdef _OPENMP
#pragma omp parallel for
#endif
  for (std::size_t i = 0; i < count; ++i) {
    state.vel_x[i] += params.external_forces.x * dt;
    state.vel_y[i] += params.external_forces.y * dt;
    state.vel_z[i] += params.external_forces.z * dt;
    scratch.pred_x[i] = state.pos_x[i] + state.vel_x[i] * dt;
    scratch.pred_y[i] = state.pos_y[i] + state.vel_y[i] * dt;
    scratch.pred_z[i] = state.pos_z[i] + state.vel_z[i] * dt;
  }

  // Build neighbor list from predicted positions.
  if (params.use_uniform_grid && h > 0.0f) {
    scratch.grid_entries.resize(count);
    for (std::size_t i = 0; i < count; ++i) {
      scratch.grid_entries[i] = CpuScratch::CellEntry{
          cell_key_from_pos(scratch.pred_x[i], scratch.pred_y[i],
                            scratch.pred_z[i], h),
          static_cast<int>(i)};
    }

    std::sort(scratch.grid_entries.begin(), scratch.grid_entries.end(),
              [](const CpuScratch::CellEntry& a,
                 const CpuScratch::CellEntry& b) {
                if (cell_key_less(a.key, b.key)) {
                  return true;
                }
                if (cell_key_less(b.key, a.key)) {
                  return false;
                }
                return a.particle < b.particle;
              });

    scratch.grid_keys.clear();
    scratch.grid_starts.clear();
    scratch.grid_ends.clear();
    if (!scratch.grid_entries.empty()) {
      CpuScratch::CellKey current = scratch.grid_entries.front().key;
      int start = 0;
      for (std::size_t i = 1; i < count; ++i) {
        if (!cell_key_equal(current, scratch.grid_entries[i].key)) {
          scratch.grid_keys.push_back(current);
          scratch.grid_starts.push_back(start);
          scratch.grid_ends.push_back(static_cast<int>(i));
          current = scratch.grid_entries[i].key;
          start = static_cast<int>(i);
        }
      }
      scratch.grid_keys.push_back(current);
      scratch.grid_starts.push_back(start);
      scratch.grid_ends.push_back(static_cast<int>(count));
    }

    for (std::size_t i = 0; i < count; ++i) {
      const float px = scratch.pred_x[i];
      const float py = scratch.pred_y[i];
      const float pz = scratch.pred_z[i];
      const CpuScratch::CellKey base = cell_key_from_pos(px, py, pz, h);

      for (int dz = -1; dz <= 1; ++dz) {
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            const CpuScratch::CellKey key{base.x + dx, base.y + dy,
                                          base.z + dz};
            auto it = std::lower_bound(
                scratch.grid_keys.begin(), scratch.grid_keys.end(), key,
                [](const CpuScratch::CellKey& a,
                   const CpuScratch::CellKey& b) {
                  return cell_key_less(a, b);
                });
            if (it == scratch.grid_keys.end() || !cell_key_equal(*it, key)) {
              continue;
            }
            const std::size_t cell_index =
                static_cast<std::size_t>(it - scratch.grid_keys.begin());
            const int start = scratch.grid_starts[cell_index];
            const int end = scratch.grid_ends[cell_index];
            for (int idx = start; idx < end; ++idx) {
              const int j = scratch.grid_entries[idx].particle;
              if (j == static_cast<int>(i)) {
                continue;
              }
              const float dxp = px - scratch.pred_x[j];
              const float dyp = py - scratch.pred_y[j];
              const float dzp = pz - scratch.pred_z[j];
              const float r2 = dxp * dxp + dyp * dyp + dzp * dzp;
              if (r2 < h2) {
                scratch.neighbor_indices.push_back(j);
              }
            }
          }
        }
      }
      scratch.neighbor_prefix_sum[i] =
          static_cast<int>(scratch.neighbor_indices.size());
    }
  } else {
    for (std::size_t i = 0; i < count; ++i) {
      const float px = scratch.pred_x[i];
      const float py = scratch.pred_y[i];
      const float pz = scratch.pred_z[i];
      for (std::size_t j = 0; j < count; ++j) {
        if (i == j) {
          continue;
        }
        const float dx = px - scratch.pred_x[j];
        const float dy = py - scratch.pred_y[j];
        const float dz = pz - scratch.pred_z[j];
        const float r2 = dx * dx + dy * dy + dz * dz;
        if (r2 < h2) {
          scratch.neighbor_indices.push_back(static_cast<int>(j));
        }
      }
      scratch.neighbor_prefix_sum[i] =
          static_cast<int>(scratch.neighbor_indices.size());
    }
  }

  const float density = params.density;
  const float inv_density = 1.0f / density;
  const float particle_mass = params.particle_mass;
  const float grad_scale = particle_mass * inv_density;
  const float epsilon = params.epsilon;

  // Solver loop: compute lambdas, then position corrections.
  for (int iter = 0; iter < params.solver_iterations; ++iter) {
#ifdef _OPENMP
#pragma omp parallel for
#endif
    for (std::size_t i = 0; i < count; ++i) {
      const int start = (i == 0) ? 0 : scratch.neighbor_prefix_sum[i - 1];
      const int end = scratch.neighbor_prefix_sum[i];
      const float px = scratch.pred_x[i];
      const float py = scratch.pred_y[i];
      const float pz = scratch.pred_z[i];

      float rho = 0.0f;
      float grad_sum_x = 0.0f;
      float grad_sum_y = 0.0f;
      float grad_sum_z = 0.0f;
      float sum_grad2 = 0.0f;

      for (int idx = start; idx < end; ++idx) {
        const int j = scratch.neighbor_indices[idx];
        const float dx = px - scratch.pred_x[j];
        const float dy = py - scratch.pred_y[j];
        const float dz = pz - scratch.pred_z[j];
        const float r2 = dx * dx + dy * dy + dz * dz;
        rho += poly6_kernel(r2, h);

        if (r2 < h2) {
          const float r = std::sqrt(r2 < min_r2 ? min_r2 : r2);
          const float grad_factor = spiky_gradient_factor(r, h);
          const float gx = grad_factor * dx;
          const float gy = grad_factor * dy;
          const float gz = grad_factor * dz;
          grad_sum_x += gx;
          grad_sum_y += gy;
          grad_sum_z += gz;
          const float grad_jx = -grad_scale * gx;
          const float grad_jy = -grad_scale * gy;
          const float grad_jz = -grad_scale * gz;
          sum_grad2 += grad_jx * grad_jx + grad_jy * grad_jy +
                       grad_jz * grad_jz;
        }
      }

      rho += poly6_kernel(0.0f, h);
      rho *= particle_mass;
      scratch.rho[i] = rho;

      const float C = rho * inv_density - 1.0f;
      const float grad_ix = grad_scale * grad_sum_x;
      const float grad_iy = grad_scale * grad_sum_y;
      const float grad_iz = grad_scale * grad_sum_z;
      sum_grad2 += grad_ix * grad_ix + grad_iy * grad_iy + grad_iz * grad_iz;
      scratch.lambda[i] = -C / (sum_grad2 + epsilon);
    }

#ifdef _OPENMP
#pragma omp parallel for
#endif
    for (std::size_t i = 0; i < count; ++i) {
      const int start = (i == 0) ? 0 : scratch.neighbor_prefix_sum[i - 1];
      const int end = scratch.neighbor_prefix_sum[i];
      const float px = scratch.pred_x[i];
      const float py = scratch.pred_y[i];
      const float pz = scratch.pred_z[i];
      const float lambda_i = scratch.lambda[i];

      float delta_x = 0.0f;
      float delta_y = 0.0f;
      float delta_z = 0.0f;

      for (int idx = start; idx < end; ++idx) {
        const int j = scratch.neighbor_indices[idx];
        const float dx = px - scratch.pred_x[j];
        const float dy = py - scratch.pred_y[j];
        const float dz = pz - scratch.pred_z[j];
        const float r2 = dx * dx + dy * dy + dz * dz;
        if (r2 < h2) {
          const float r = std::sqrt(r2 < min_r2 ? min_r2 : r2);
          const float grad_factor = spiky_gradient_factor(r, h);
          float s = lambda_i + scratch.lambda[j];
          if (scorr_enabled && scorr_inv_wdq > 0.0f) {
            const float W = poly6_kernel(r2, h);
            const float ratio = W * scorr_inv_wdq;
            const float scorr = -params.scorr_k * pow_ratio_n(ratio, params.scorr_n);
            s += scorr;
          }
          delta_x += s * grad_factor * dx;
          delta_y += s * grad_factor * dy;
          delta_z += s * grad_factor * dz;
        }
      }

      delta_x *= inv_density;
      delta_y *= inv_density;
      delta_z *= inv_density;

      const std::size_t plane_count = params.planes.size();
      if (plane_count > 0) {
        float pred_x = px + delta_x;
        float pred_y = py + delta_y;
        float pred_z = pz + delta_z;
        for (std::size_t p = 0; p < plane_count; ++p) {
          const float nx = params.planes.nx[p];
          const float ny = params.planes.ny[p];
          const float nz = params.planes.nz[p];
          const float d = params.planes.d[p];
          const float sd = nx * pred_x + ny * pred_y + nz * pred_z - d;
          const float penetration = -sd;
          if (penetration > 0.0f) {
            pred_x += nx * penetration;
            pred_y += ny * penetration;
            pred_z += nz * penetration;
          }
        }
        delta_x = pred_x - px;
        delta_y = pred_y - py;
        delta_z = pred_z - pz;
      }

      scratch.delta_x[i] = delta_x;
      scratch.delta_y[i] = delta_y;
      scratch.delta_z[i] = delta_z;
    }

#ifdef _OPENMP
#pragma omp parallel for
#endif
    for (std::size_t i = 0; i < count; ++i) {
      scratch.pred_x[i] += scratch.delta_x[i];
      scratch.pred_y[i] += scratch.delta_y[i];
      scratch.pred_z[i] += scratch.delta_z[i];
    }
  }

  // Update velocities and commit predicted positions.
#ifdef _OPENMP
#pragma omp parallel for
#endif
  for (std::size_t i = 0; i < count; ++i) {
    state.vel_x[i] = (scratch.pred_x[i] - state.pos_x[i]) / dt;
    state.vel_y[i] = (scratch.pred_y[i] - state.pos_y[i]) / dt;
    state.vel_z[i] = (scratch.pred_z[i] - state.pos_z[i]) / dt;
    state.pos_x[i] = scratch.pred_x[i];
    state.pos_y[i] = scratch.pred_y[i];
    state.pos_z[i] = scratch.pred_z[i];
  }

  if (params.enable_xsph && params.visc_c != 0.0f) {
    std::fill(scratch.dv_x.begin(), scratch.dv_x.end(), 0.0f);
    std::fill(scratch.dv_y.begin(), scratch.dv_y.end(), 0.0f);
    std::fill(scratch.dv_z.begin(), scratch.dv_z.end(), 0.0f);
#ifdef _OPENMP
#pragma omp parallel for
#endif
    for (std::size_t i = 0; i < count; ++i) {
      const int start = (i == 0) ? 0 : scratch.neighbor_prefix_sum[i - 1];
      const int end = scratch.neighbor_prefix_sum[i];
      const float vx = state.vel_x[i];
      const float vy = state.vel_y[i];
      const float vz = state.vel_z[i];
      float dvx = 0.0f;
      float dvy = 0.0f;
      float dvz = 0.0f;
      for (int idx = start; idx < end; ++idx) {
        const int j = scratch.neighbor_indices[idx];
        const float dx = state.pos_x[i] - state.pos_x[j];
        const float dy = state.pos_y[i] - state.pos_y[j];
        const float dz = state.pos_z[i] - state.pos_z[j];
        const float r2 = dx * dx + dy * dy + dz * dz;
        if (r2 < h2) {
          const float W = poly6_kernel(r2, h);
          const float inv_rho_j =
              (scratch.rho[j] > 0.0f) ? (particle_mass / scratch.rho[j]) : 0.0f;
          dvx += (state.vel_x[j] - vx) * W * inv_rho_j;
          dvy += (state.vel_y[j] - vy) * W * inv_rho_j;
          dvz += (state.vel_z[j] - vz) * W * inv_rho_j;
        }
      }
      scratch.dv_x[i] = dvx;
      scratch.dv_y[i] = dvy;
      scratch.dv_z[i] = dvz;
    }
#ifdef _OPENMP
#pragma omp parallel for
#endif
    for (std::size_t i = 0; i < count; ++i) {
      state.vel_x[i] += params.visc_c * scratch.dv_x[i];
      state.vel_y[i] += params.visc_c * scratch.dv_y[i];
      state.vel_z[i] += params.visc_c * scratch.dv_z[i];
    }
  }

  if (params.enable_vorticity && params.vort_epsilon != 0.0f) {
#ifdef _OPENMP
#pragma omp parallel for
#endif
    for (std::size_t i = 0; i < count; ++i) {
      const int start = (i == 0) ? 0 : scratch.neighbor_prefix_sum[i - 1];
      const int end = scratch.neighbor_prefix_sum[i];
      const float px = state.pos_x[i];
      const float py = state.pos_y[i];
      const float pz = state.pos_z[i];
      const float vx = state.vel_x[i];
      const float vy = state.vel_y[i];
      const float vz = state.vel_z[i];
      float ox = 0.0f;
      float oy = 0.0f;
      float oz = 0.0f;
      for (int idx = start; idx < end; ++idx) {
        const int j = scratch.neighbor_indices[idx];
        const float dx = px - state.pos_x[j];
        const float dy = py - state.pos_y[j];
        const float dz = pz - state.pos_z[j];
        const float r2 = dx * dx + dy * dy + dz * dz;
        if (r2 < h2) {
          const float r = std::sqrt(r2 < min_r2 ? min_r2 : r2);
          const float grad_factor = spiky_gradient_factor(r, h);
          const float gx = grad_factor * dx;
          const float gy = grad_factor * dy;
          const float gz = grad_factor * dz;
          const float dvx = state.vel_x[j] - vx;
          const float dvy = state.vel_y[j] - vy;
          const float dvz = state.vel_z[j] - vz;
          ox += dvy * gz - dvz * gy;
          oy += dvz * gx - dvx * gz;
          oz += dvx * gy - dvy * gx;
        }
      }
      scratch.omega_x[i] = ox;
      scratch.omega_y[i] = oy;
      scratch.omega_z[i] = oz;
      scratch.omega_mag[i] = std::sqrt(ox * ox + oy * oy + oz * oz);
    }
#ifdef _OPENMP
#pragma omp parallel for
#endif
    for (std::size_t i = 0; i < count; ++i) {
      const int start = (i == 0) ? 0 : scratch.neighbor_prefix_sum[i - 1];
      const int end = scratch.neighbor_prefix_sum[i];
      const float px = state.pos_x[i];
      const float py = state.pos_y[i];
      const float pz = state.pos_z[i];
      const float omi = scratch.omega_mag[i];
      float ex = 0.0f;
      float ey = 0.0f;
      float ez = 0.0f;
      for (int idx = start; idx < end; ++idx) {
        const int j = scratch.neighbor_indices[idx];
        const float dx = px - state.pos_x[j];
        const float dy = py - state.pos_y[j];
        const float dz = pz - state.pos_z[j];
        const float r2 = dx * dx + dy * dy + dz * dz;
        if (r2 < h2) {
          const float r = std::sqrt(r2 < min_r2 ? min_r2 : r2);
          const float grad_factor = spiky_gradient_factor(r, h);
          const float gx = grad_factor * dx;
          const float gy = grad_factor * dy;
          const float gz = grad_factor * dz;
          const float coeff = scratch.omega_mag[j] - omi;
          ex += coeff * gx;
          ey += coeff * gy;
          ez += coeff * gz;
        }
      }
      scratch.eta_x[i] = ex;
      scratch.eta_y[i] = ey;
      scratch.eta_z[i] = ez;
    }
#ifdef _OPENMP
#pragma omp parallel for
#endif
    for (std::size_t i = 0; i < count; ++i) {
      const float ex = scratch.eta_x[i];
      const float ey = scratch.eta_y[i];
      const float ez = scratch.eta_z[i];
      const float eta_len = std::sqrt(ex * ex + ey * ey + ez * ez);
      float nx = 0.0f;
      float ny = 0.0f;
      float nz = 0.0f;
      if (eta_len > params.vort_norm_eps) {
        const float inv = 1.0f / eta_len;
        nx = ex * inv;
        ny = ey * inv;
        nz = ez * inv;
      }
      const float ox = scratch.omega_x[i];
      const float oy = scratch.omega_y[i];
      const float oz = scratch.omega_z[i];
      const float fx = params.vort_epsilon * (ny * oz - nz * oy);
      const float fy = params.vort_epsilon * (nz * ox - nx * oz);
      const float fz = params.vort_epsilon * (nx * oy - ny * ox);
      state.vel_x[i] += dt * fx;
      state.vel_y[i] += dt * fy;
      state.vel_z[i] += dt * fz;
    }
  }

  if (params.plane_restitution > 0.0f || params.plane_friction > 0.0f) {
    const std::size_t plane_count = params.planes.size();
    if (plane_count > 0) {
#ifdef _OPENMP
#pragma omp parallel for
#endif
      for (std::size_t i = 0; i < count; ++i) {
        float vx = state.vel_x[i];
        float vy = state.vel_y[i];
        float vz = state.vel_z[i];
        const float px = state.pos_x[i];
        const float py = state.pos_y[i];
        const float pz = state.pos_z[i];
        for (std::size_t p = 0; p < plane_count; ++p) {
          const float nx = params.planes.nx[p];
          const float ny = params.planes.ny[p];
          const float nz = params.planes.nz[p];
          const float d = params.planes.d[p];
          const float sd = nx * px + ny * py + nz * pz - d;
          if (sd <= 0.0f) {
            const float vn = nx * vx + ny * vy + nz * vz;
            float vn_new = vn;
            if (vn < 0.0f) {
              vn_new = -params.plane_restitution * vn;
            }
            const float vt_x = vx - vn * nx;
            const float vt_y = vy - vn * ny;
            const float vt_z = vz - vn * nz;
            const float vt_scale = 1.0f - params.plane_friction;
            vx = vt_x * vt_scale + vn_new * nx;
            vy = vt_y * vt_scale + vn_new * ny;
            vz = vt_z * vt_scale + vn_new * nz;
          }
        }
        state.vel_x[i] = vx;
        state.vel_y[i] = vy;
        state.vel_z[i] = vz;
      }
    }
  }

  state.time += dt;
}

}  // namespace fluid
