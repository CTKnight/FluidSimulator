#ifdef BUILD_CUDA

#include <thrust/device_vector.h>
#include <memory>

#include "../main.h"
#include "cudaFluid.cuh"
#include "cudaPlane.cuh"

using namespace std;


int main(int argc, char **argv) {
  // Attempt to find project root
  std::vector<std::string> search_paths = {
    ".",
    "..",
    "../..",
    "../../.."
  };
  std::string project_root;
  bool found_project_root = find_project_root(search_paths, project_root);
  
  shared_ptr<Fluid> fluid;
  shared_ptr<FluidParameters> fp = make_shared<FluidParameters>();
  vector<CollisionObject *> objects_std;
  
  int c;
  
  std::string file_to_load_from;
  bool file_specified = false;

  std::string particle_foldername_to_output;
  int sec = 1;
  constexpr int frames_per_sec = 30;
  constexpr int simulation_steps = 4;
  
  while ((c = getopt (argc, argv, "f:r:p:s:")) != -1) {
    switch (c) {
      case 'f': {
        file_to_load_from = optarg;
        file_specified = true;
        break;
      }
      case 'r': {
        project_root = optarg;
        if (!is_valid_project_root(project_root)) {
          std::cout << "Warn: Could not find required file \"shaders/Default.vert\" in specified project root: " << project_root << std::endl;
        }
        found_project_root = true;
        break;
      }
      case 'p': {
        particle_foldername_to_output = optarg;
        break;
      }
      case 's': {
        sec = atoi(optarg);
        if (sec < 1) {
          sec = 1;
        }
        break;
      }
      default: {
        usageErrorCuda(argv[0]);
        break;
      }
    }
  }

  bool particle_folder_to_output_good = false;
  if (particle_foldername_to_output.length() != 0) {
    int err = mkdir_main(particle_foldername_to_output.c_str());
    particle_folder_to_output_good = err == 0;
    if (!particle_folder_to_output_good) {
      std::cout << "Warn: can't mkdir at " << particle_foldername_to_output << ", will not write to it\n";
    }
  }

  bool success = loadObjectsFromFile(file_to_load_from, fluid, fp, &objects_std);
  
  if (!success) {
    std::cout << "Warn: Unable to load from file: " << file_to_load_from << std::endl;
    exit(-1);
  }

  unique_ptr<vector<REAL3>> position_fluid_cuda = make_unique<vector<REAL3>>(fluid->getParticlePositions());
  unique_ptr<vector<REAL3>> velocities_fluid_cuda = make_unique<vector<REAL3>>(fluid->getParticleVelocities());
  shared_ptr<Fluid_cuda> fluid_cuda = make_shared<Fluid_cuda>(std::move(position_fluid_cuda), std::move(velocities_fluid_cuda), fp->h);
  fluid_cuda->init();
  thrust::device_vector<Plane_cuda> collision_planes;
  collision_planes.resize(objects_std.size());
  for (int i = 0; i < objects_std.size(); i++) {
    const auto co = objects_std[i];
    if (Plane *p = dynamic_cast<Plane *>(co)) {
      collision_planes[i] = Plane_cuda(*p);
    } else {
      std::cout << "CUDA version only support plane collision, omitting this collision object.\n";
    }
  }

  REAL delta_t = 1. / frames_per_sec / simulation_steps;

  int n = 0;
  if (particle_folder_to_output_good) {
    writeFluidToFileN(particle_foldername_to_output, n, *fluid_cuda);
  }
  n++;
  for (int i = 0; i < sec; i++) {
    for (int frame = 0; frame < frames_per_sec; frame++) {
      const auto start = chrono::high_resolution_clock::now(); 
      for (int i = 0; i < simulation_steps; i++) {
        fluid_cuda->simulate(delta_t, fp.get(), collision_planes);
      }
      const auto end = chrono::high_resolution_clock::now();
      const auto duration = chrono::duration_cast<chrono::microseconds>(end-start);
      cout << n << " frame" << ", simulated for " << simulation_steps << " steps in " << duration.count() << " microsec." << endl; 
      if (particle_folder_to_output_good) {
        writeFluidToFileN(particle_foldername_to_output, n, *fluid_cuda);
      }
      n++;
    }
  }
  if (particle_folder_to_output_good) {
    writeFluidSeriesPvd(particle_foldername_to_output, n, frames_per_sec);
  }
  return 0;  
}

#endif
