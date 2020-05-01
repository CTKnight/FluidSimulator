#include "../main.h"
#include "cudaFluid.h"
#include "../collision/plane.h"
#include "../collision/sphere.h"

using namespace std;

#ifdef BUILD_CUDA

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
  vector<CollisionObject *> objects;
  
  int c;
  
  std::string file_to_load_from;
  bool file_specified = false;

  std::string particle_foldername_to_output;
  int sec = 1;
  
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

  

  return 0;  
}

#endif