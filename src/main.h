#ifndef MAIN_H
#define MAIN_H

#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <unordered_set>
#include <cerrno>
#include <cctype>
#include <stdlib.h> // atoi for getopt inputs

#ifdef _WIN32
#include "misc/getopt.h" // getopt for windows
#include <direct.h> // _mkdir
#else
#include <getopt.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#endif

#include "collision/plane.h"
#include "collision/sphere.h"
#include "fluid.h"
#include "fluid_io.h"
#include "misc/file_utils.h"
#include "real.h"

#ifdef BUILD_CUDA
#include "cuda/cudaFluid.cuh"
#endif

#include "json.hpp"
using json = nlohmann::json;
using namespace std;

const string PLANE = "plane";
const string FLUID = "fluid";
const string COLLISIONS = "collisions";
const string EXTERNAL_FORCES = "external_forces";

const unordered_set<string> VALID_KEYS = {FLUID, COLLISIONS};

void usageError(const char *binaryName) {
  printf("Usage: %s [options]\n", binaryName);
  printf("Required program options:\n");
  printf("  -f     <STRING>    Filename of scene\n");
  printf("  -r     <STRING>    Project root.\n");
  printf("                     Should contain \"shaders/Default.vert\".\n");
  printf("                     Automatically searched for by default.\n");
  printf("  -a     <INT>       Sphere vertices latitude direction.\n");
  printf("  -o     <INT>       Sphere vertices longitude direction.\n");
  printf("  -p     <STRING>    Output particle data folder.\n");
  printf("  -i     <STRING>    Input particle data folder.\n");
  printf("  -n                 off for no window.\n");
  printf("  -s     <INT>       No window rendering duration.\n");
  printf("\n");
  exit(-1);
}

void usageErrorCuda(const char *binaryName) {
  printf("Usage: %s [options]\n", binaryName);
  printf("Required program options:\n");
  printf("  -f     <STRING>    Filename of scene\n");
  printf("  -r     <STRING>    Project root.\n");
  printf("                     Should contain \"shaders/Default.vert\".\n");
  printf("                     Automatically searched for by default.\n");
  printf("  -p     <STRING>    Output particle data folder.\n");
  printf("  -s     <INT>       rendering duration.\n");
  printf("\n");
  exit(-1);
}

void incompleteObjectError(const char *object, const char *attribute) {
  cout << "Incomplete " << object << " definition, missing " << attribute << endl;
  exit(-1);
}

bool loadObjectsFromFile(string filename, shared_ptr<Fluid> &fluid, shared_ptr<FluidParameters> &fp, vector<CollisionObject *>* objects) {
  std::cout << "loadObjectsFromFile: " << filename << "\n";

  // Read JSON from file
  ifstream i(filename);
  if (!i.good()) {
    return false;
  }
  json j;
  i >> j;

  json object = j[FLUID];
  REAL particle_mass = object["particle_mass"];
  REAL density = object["density"];
  REAL cube_size_per_particle = 1. / pow(density/particle_mass, 1./3.);
  REAL h = object["h"];
  REAL epsilon = object["epsilon"];
  REAL n = object["n"];
  REAL k = object["k"];
  REAL c = object["c"];
  auto shape = object["shape"];
  if (!shape.is_array()) {
    throw std::runtime_error("Fluid shape should be an array");
  }

  unique_ptr<vector<REAL3>> particles = make_unique<vector<REAL3>>();
  unique_ptr<vector<REAL3>> velocites = make_unique<vector<REAL3>>();
  for (const auto &el: shape) {
    string type = el["type"];
    int particleNum = 0;
    REAL3 velocity = {0,0,0};
    if (el.find("velocity") != el.end()) {
      vector<REAL> velocity_vec = el["velocity"];
      for (int i = 0; i < 3; i++) {
        velocity[i] = velocity_vec[i];
      }
    }
    if (type == "cube") {
      vector<REAL> origin = el["origin"];
      vector<REAL> size = el["size"];
      for (REAL i = origin[0] + cube_size_per_particle/2; i < origin[0] + size[0]; i += cube_size_per_particle) {
        for (REAL j = origin[1] + cube_size_per_particle/2; j < origin[1] + size[1]; j += cube_size_per_particle) {
          for (REAL k = origin[2] + cube_size_per_particle/2; k < origin[2] + size[2]; k += cube_size_per_particle) {
            particles->emplace_back(REAL3{i, j, k});
            velocites->emplace_back(velocity);
          }
        }
      }
    } else if (type == "sphere") {
      vector<REAL> origin = el["origin"];
      REAL radius = el["radius"];
      REAL r2 = pow(radius, 2);
      for (REAL i = origin[0]-radius + cube_size_per_particle/2; i < origin[0]+radius; i += cube_size_per_particle) {
        for (REAL j = origin[1]-radius + cube_size_per_particle/2; j < origin[1]+radius; j += cube_size_per_particle) {
          for (REAL k = origin[2]-radius + cube_size_per_particle/2; k < origin[2]+radius; k += cube_size_per_particle) {
            if (pow(i-origin[0], 2) + pow(j-origin[1], 2) + pow(k-origin[2], 2)< r2) {
              particles->emplace_back(REAL3{i, j, k});
              velocites->emplace_back(velocity);
            }
          }
        }
      }
    } else if (type == "file") {
      string path = el["path"];
      // TODO
    } else {
      throw std::runtime_error(string("Invalid fluid.shape type: ") + type);
    }
  }
  // h: SPH Basics p16
  fluid = make_shared<Fluid>(std::move(particles), std::move(velocites), h);
  fp = make_shared<FluidParameters>(density, particle_mass, 0.1, h, epsilon, n, k, c);

  object = j[COLLISIONS];
  if (!object.is_array()) {
    throw std::runtime_error(COLLISIONS + std::string(" should be an array"));
  }
  for (const auto &el: object) {
    const string type = el["type"];
    CollisionObject *p = nullptr;
    if (type == string("plane")) {
      vector<REAL> vec_point = el["point"];
      vector<REAL> vec_normal = el["normal"];
      REAL friction = el["friction"];
      Vector3R point{vec_point[0], vec_point[1], vec_point[2]};
      Vector3R normal{vec_normal[0], vec_normal[1], vec_normal[2]};
      p = new Plane(point, normal, friction);
    } else if (type == string("sphere")) {
      vector<REAL> origin = el["origin"];
      REAL radius = el["radius"];
      REAL friction = el["friction"];
      p = new Sphere(Vector3R(origin[0],origin[1],origin[2]), radius, friction);
    }
    if (p) {
      objects->push_back(p);
    }
  }

  object = j[EXTERNAL_FORCES];
  if (object.is_array()) {
    vector<REAL> external_forces_vec = object;
    for (int i = 0; i < 3; i++) {
      fp->external_forces[i] = external_forces_vec[i];
    }
  }

  i.close();
  
  return true;
}

bool is_valid_project_root(const std::string& search_path) {
    std::stringstream ss;
    ss << search_path;
    ss << "/";
    ss << "shaders/Default.vert";
    
    return FileUtils::file_exists(ss.str());
}

// Attempt to locate the project root automatically
bool find_project_root(const std::vector<std::string>& search_paths, std::string& retval) {
  
  for (std::string search_path : search_paths) {
    if (is_valid_project_root(search_path)) {
      retval = search_path;
      return true;
    }
  }
  return false;
}

int mkdir_main(const char *dir) {
  if (dir == nullptr || *dir == '\0') {
    return -1;
  }

  std::string path(dir);
  std::string current;
  size_t start = 0;

  if (path.size() >= 2 && std::isalpha(static_cast<unsigned char>(path[0])) && path[1] == ':') {
    current = path.substr(0, 2);
    start = 2;
    if (path.size() > 2 && (path[2] == '/' || path[2] == '\\')) {
      current += "/";
      start = 3;
    }
  } else if (path[0] == '/' || path[0] == '\\') {
    current = "/";
    start = 1;
  }

  while (start < path.size()) {
    const size_t sep = path.find_first_of("/\\", start);
    const std::string part = path.substr(start, sep - start);
    if (!part.empty()) {
      if (!current.empty() && current.back() != '/' && current.back() != '\\') {
        current += "/";
      }
      current += part;
      errno = 0;
      // from https://stackoverflow.com/questions/23427804/cant-find-mkdir-function-in-dirent-h-for-windows
      #ifdef _WIN32
        const int err = _mkdir(current.c_str());
      #else
        const int err = mkdir(current.c_str(), 0755);
      #endif
      if (err != 0) {
        if (errno != EEXIST || !FileUtils::directory_exists(current)) {
          return -1;
        }
      }
    }
    if (sep == std::string::npos) {
      break;
    }
    start = sep + 1;
  }

  return 0;
}

void writeFluidToFileN(const string &particle_foldername_to_output, int n, const Fluid &fluid) {
  const auto output_filename = FileUtils::fluid_filename(particle_foldername_to_output, n);
  ofstream particle_file_to_output(output_filename);
  if (particle_file_to_output) {
    particle_file_to_output << fluid;
  } else {
    throw std::runtime_error(output_filename + string(" is not good to write!"));
  }
  particle_file_to_output.close();
}
#ifdef BUILD_CUDA
void writeFluidToFileN(const string &particle_foldername_to_output, int n, const Fluid_cuda &fluid) {
  const auto output_filename = FileUtils::fluid_filename(particle_foldername_to_output, n);
  ofstream particle_file_to_output(output_filename);
  if (particle_file_to_output) {
    particle_file_to_output << fluid;
  } else {
    throw std::runtime_error(output_filename + string(" is not good to write!"));
  }
  particle_file_to_output.close();
}
#endif

#endif
