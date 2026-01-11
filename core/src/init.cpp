#include "fluid/init.h"

#include <cmath>
#include <fstream>
#include <sstream>

#include "fluid/third_party/json.hpp"

namespace fluid {

namespace {

using json = nlohmann::json;

float default_spacing(const Params& params) {
  if (params.density <= 0.0f || params.particle_mass <= 0.0f) {
    return 0.0f;
  }
  const float volume_per_particle = params.particle_mass / params.density;
  return std::cbrt(volume_per_particle);
}

bool read_vec3(const json& value,
               float& x,
               float& y,
               float& z,
               std::string* error,
               const char* label) {
  if (!value.is_array() || value.size() != 3) {
    if (error) {
      *error = std::string("Expected vec3 for ") + label;
    }
    return false;
  }
  try {
    x = value.at(0).get<float>();
    y = value.at(1).get<float>();
    z = value.at(2).get<float>();
  } catch (const std::exception& ex) {
    if (error) {
      *error = std::string("Invalid vec3 for ") + label + ": " + ex.what();
    }
    return false;
  }
  return true;
}

void set_error(std::string* error, const std::string& message) {
  if (error) {
    *error = message;
  }
}

bool has_key(const json& value, const char* key) {
  if (!value.is_object()) {
    return false;
  }
  return value.find(key) != value.end();
}

std::size_t count_from_extent(float extent, float spacing) {
  if (extent <= 0.0f || spacing <= 0.0f) {
    return 0;
  }
  const std::size_t count =
      static_cast<std::size_t>(std::floor(extent / spacing));
  return count > 0 ? count : 1;
}

}  // namespace

void init_block(const Params& params,
                State& state,
                std::size_t nx,
                std::size_t ny,
                std::size_t nz,
                float origin_x,
                float origin_y,
                float origin_z,
                float spacing) {
  const std::size_t count = nx * ny * nz;
  state.pos_x.assign(count, 0.0f);
  state.pos_y.assign(count, 0.0f);
  state.pos_z.assign(count, 0.0f);
  state.vel_x.assign(count, 0.0f);
  state.vel_y.assign(count, 0.0f);
  state.vel_z.assign(count, 0.0f);
  state.time = 0.0f;
  state.cpu.resize(count);

  float step = spacing;
  if (step <= 0.0f) {
    step = default_spacing(params);
  }

  std::size_t index = 0;
  for (std::size_t z = 0; z < nz; ++z) {
    for (std::size_t y = 0; y < ny; ++y) {
      for (std::size_t x = 0; x < nx; ++x) {
        state.pos_x[index] = origin_x + static_cast<float>(x) * step;
        state.pos_y[index] = origin_y + static_cast<float>(y) * step;
        state.pos_z[index] = origin_z + static_cast<float>(z) * step;
        index++;
      }
    }
  }
}

void init_planes(Params& params, float box_x, float box_y, float box_z) {
  params.planes.clear();
  params.planes.add_normalized(1.0f, 0.0f, 0.0f, 0.0f);
  params.planes.add_normalized(-1.0f, 0.0f, 0.0f, -box_x);
  params.planes.add_normalized(0.0f, 1.0f, 0.0f, 0.0f);
  params.planes.add_normalized(0.0f, -1.0f, 0.0f, -box_y);
  params.planes.add_normalized(0.0f, 0.0f, 1.0f, 0.0f);
  params.planes.add_normalized(0.0f, 0.0f, -1.0f, -box_z);
}

void init_test_scene(Params& params, State& state) {
  const float box_x = 1.0f;
  const float box_y = 3.0f;
  const float box_z = 1.0f;
  init_planes(params, box_x, box_y, box_z);

  float spacing = 0.0f;
  if (params.particle_radius > 0.0f) {
    spacing = params.particle_radius * 2.0f;
  }
  if (spacing <= 0.0f) {
    spacing = default_spacing(params);
  }
  if (spacing <= 0.0f) {
    spacing = 0.02f;
  }

  if (params.density > 0.0f) {
    params.particle_mass = params.density * spacing * spacing * spacing;
  }
  params.h = 2.5f * spacing;

  const float block_x = box_x * 0.5f;
  const float block_y = box_y * 0.5f;
  const float block_z = box_z * 0.5f;

  const std::size_t nx = count_from_extent(block_x, spacing);
  const std::size_t ny = count_from_extent(block_y, spacing);
  const std::size_t nz = count_from_extent(block_z, spacing);

  const float block_size_x = static_cast<float>(nx) * spacing;
  const float block_size_z = static_cast<float>(nz) * spacing;
  const float origin_x = 0.5f * (box_x - block_size_x);
  const float origin_y = 0.5f * box_y;
  const float origin_z = 0.5f * (box_z - block_size_z);

  init_block(params, state, nx, ny, nz, origin_x, origin_y, origin_z, spacing);
}

bool init_scene_from_json(const std::string& path,
                          Params& params,
                          State& state,
                          std::string* error) {
  std::ifstream in(path);
  if (!in) {
    set_error(error, "Failed to open scene file: " + path);
    return false;
  }

  json root;
  try {
    in >> root;
  } catch (const std::exception& ex) {
    set_error(error, std::string("Failed to parse JSON: ") + ex.what());
    return false;
  }

  if (!has_key(root, "fluid") || !root["fluid"].is_object()) {
    set_error(error, "Scene JSON missing fluid object.");
    return false;
  }

  const json& fluid = root["fluid"];
  try {
    if (has_key(fluid, "particle_mass")) {
      params.particle_mass = fluid["particle_mass"].get<float>();
    }
    if (has_key(fluid, "density")) {
      params.density = fluid["density"].get<float>();
    }
    if (has_key(fluid, "h")) {
      params.h = fluid["h"].get<float>();
    }
    if (has_key(fluid, "epsilon")) {
      params.epsilon = fluid["epsilon"].get<float>();
    }
    if (has_key(fluid, "n")) {
      params.scorr_n = fluid["n"].get<int>();
    }
    if (has_key(fluid, "k")) {
      params.scorr_k = fluid["k"].get<float>();
    }
    if (has_key(fluid, "c")) {
      params.visc_c = fluid["c"].get<float>();
    }
  } catch (const std::exception& ex) {
    set_error(error, std::string("Invalid fluid parameters: ") + ex.what());
    return false;
  }

  if (has_key(root, "external_forces")) {
    float fx = 0.0f;
    float fy = 0.0f;
    float fz = 0.0f;
    if (!read_vec3(root["external_forces"], fx, fy, fz, error,
                   "external_forces")) {
      return false;
    }
    params.external_forces.x = fx;
    params.external_forces.y = fy;
    params.external_forces.z = fz;
  }

  float spacing = 0.0f;
  bool spacing_from_mass = false;
  if (params.density > 0.0f && params.particle_mass > 0.0f) {
    spacing = std::cbrt(params.particle_mass / params.density);
    spacing_from_mass = true;
  }
  if (spacing <= 0.0f && params.particle_radius > 0.0f) {
    spacing = params.particle_radius * 2.0f;
  }
  if (spacing <= 0.0f) {
    spacing = 0.02f;
  }

  if (spacing_from_mass) {
    params.particle_radius = spacing * 0.5f;
  } else if (params.particle_radius <= 0.0f) {
    params.particle_radius = spacing * 0.5f;
  }
  if (params.h <= 0.0f) {
    params.h = 2.5f * spacing;
  }

  if (!has_key(fluid, "shape") || !fluid["shape"].is_array()) {
    set_error(error, "Fluid shape must be an array.");
    return false;
  }

  std::vector<float> pos_x;
  std::vector<float> pos_y;
  std::vector<float> pos_z;
  std::vector<float> vel_x;
  std::vector<float> vel_y;
  std::vector<float> vel_z;

  for (const auto& shape : fluid["shape"]) {
    if (!shape.is_object()) {
      set_error(error, "Fluid shape entry must be an object.");
      return false;
    }
    if (!has_key(shape, "type")) {
      set_error(error, "Fluid shape entry missing type.");
      return false;
    }
    std::string type;
    try {
      type = shape.at("type").get<std::string>();
    } catch (const std::exception& ex) {
      set_error(error, std::string("Invalid shape type: ") + ex.what());
      return false;
    }
    float vx = 0.0f;
    float vy = 0.0f;
    float vz = 0.0f;
    if (has_key(shape, "velocity")) {
      if (!read_vec3(shape.at("velocity"), vx, vy, vz, error, "shape.velocity")) {
        return false;
      }
    }

    if (type == "cube") {
      float ox = 0.0f;
      float oy = 0.0f;
      float oz = 0.0f;
      float sx = 0.0f;
      float sy = 0.0f;
      float sz = 0.0f;
      if (!has_key(shape, "origin") || !has_key(shape, "size")) {
        set_error(error, "Cube shape missing origin or size.");
        return false;
      }
      if (!read_vec3(shape.at("origin"), ox, oy, oz, error, "shape.origin")) {
        return false;
      }
      if (!read_vec3(shape.at("size"), sx, sy, sz, error, "shape.size")) {
        return false;
      }
      for (float x = ox + spacing * 0.5f; x < ox + sx; x += spacing) {
        for (float y = oy + spacing * 0.5f; y < oy + sy; y += spacing) {
          for (float z = oz + spacing * 0.5f; z < oz + sz; z += spacing) {
            pos_x.push_back(x);
            pos_y.push_back(y);
            pos_z.push_back(z);
            vel_x.push_back(vx);
            vel_y.push_back(vy);
            vel_z.push_back(vz);
          }
        }
      }
    } else if (type == "sphere") {
      float ox = 0.0f;
      float oy = 0.0f;
      float oz = 0.0f;
      if (!has_key(shape, "origin") || !has_key(shape, "radius")) {
        set_error(error, "Sphere shape missing origin or radius.");
        return false;
      }
      if (!read_vec3(shape.at("origin"), ox, oy, oz, error, "shape.origin")) {
        return false;
      }
      float radius = 0.0f;
      try {
        radius = shape.at("radius").get<float>();
      } catch (const std::exception& ex) {
        set_error(error, std::string("Invalid sphere radius: ") + ex.what());
        return false;
      }
      const float r2 = radius * radius;
      for (float x = ox - radius + spacing * 0.5f; x < ox + radius;
           x += spacing) {
        for (float y = oy - radius + spacing * 0.5f; y < oy + radius;
             y += spacing) {
          for (float z = oz - radius + spacing * 0.5f; z < oz + radius;
               z += spacing) {
            const float dx = x - ox;
            const float dy = y - oy;
            const float dz = z - oz;
            if (dx * dx + dy * dy + dz * dz < r2) {
              pos_x.push_back(x);
              pos_y.push_back(y);
              pos_z.push_back(z);
              vel_x.push_back(vx);
              vel_y.push_back(vy);
              vel_z.push_back(vz);
            }
          }
        }
      }
    } else {
      set_error(error, "Unsupported shape type: " + type);
      return false;
    }
  }

  state.pos_x = std::move(pos_x);
  state.pos_y = std::move(pos_y);
  state.pos_z = std::move(pos_z);
  state.vel_x = std::move(vel_x);
  state.vel_y = std::move(vel_y);
  state.vel_z = std::move(vel_z);
  state.time = 0.0f;
  state.cpu.resize(state.size());

  params.planes.clear();
  if (has_key(root, "collisions")) {
    const json& collisions = root["collisions"];
    if (!collisions.is_array()) {
      set_error(error, "collisions must be an array.");
      return false;
    }
    for (const auto& entry : collisions) {
      if (!entry.is_object() || !has_key(entry, "type")) {
        set_error(error, "Collision entry missing type.");
        return false;
      }
      std::string type;
      try {
        type = entry.at("type").get<std::string>();
      } catch (const std::exception& ex) {
        set_error(error, std::string("Invalid collision type: ") + ex.what());
        return false;
      }
      if (type != "plane") {
        set_error(error, "Unsupported collision type: " + type);
        return false;
      }
      float px = 0.0f;
      float py = 0.0f;
      float pz = 0.0f;
      float nx = 0.0f;
      float ny = 0.0f;
      float nz = 0.0f;
      if (!has_key(entry, "point") || !has_key(entry, "normal")) {
        set_error(error, "Plane collision missing point or normal.");
        return false;
      }
      if (!read_vec3(entry.at("point"), px, py, pz, error, "collision.point")) {
        return false;
      }
      if (!read_vec3(entry.at("normal"), nx, ny, nz, error, "collision.normal")) {
        return false;
      }
      const float len_sq = nx * nx + ny * ny + nz * nz;
      if (len_sq <= 0.0f) {
        set_error(error, "Collision normal must be non-zero.");
        return false;
      }
      const float inv_len = 1.0f / std::sqrt(len_sq);
      const float nxn = nx * inv_len;
      const float nyn = ny * inv_len;
      const float nzn = nz * inv_len;
      const float d = nxn * px + nyn * py + nzn * pz;
      params.planes.add(nxn, nyn, nzn, d);
    }
  }

  return true;
}

}  // namespace fluid
