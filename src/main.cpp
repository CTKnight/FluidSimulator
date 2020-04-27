#include <iostream>
#include <fstream>
#include <nanogui/nanogui.h>
#include <stdio.h>
#include <stdlib.h>
#ifdef _WIN32
#include "misc/getopt.h" // getopt for windows
#include <direct.h> // _mkdir
#else
#include <getopt.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#endif
#include <unordered_set>
#include <stdlib.h> // atoi for getopt inputs


#include "CGL/CGL.h"
#include "collision/plane.h"
#include "collision/sphere.h"
#include "cloth.h"
#include "clothSimulator.h"
#include "json.hpp"
#include "misc/file_utils.h"

typedef uint32_t gid_t;

using namespace std;
using namespace nanogui;

using json = nlohmann::json;

#define msg(s) cerr << "[ClothSim] " << s << endl;

const string PLANE = "plane";
const string FLUID = "fluid";
const string COLLISIONS = "collisions";
const string EXTERNAL_FORCES = "external_forces";

const unordered_set<string> VALID_KEYS = {FLUID, COLLISIONS};

ClothSimulator *app = nullptr;
GLFWwindow *window = nullptr;
Screen *screen = nullptr;

void error_callback(int error, const char* description) {
  puts(description);
}

void createGLContexts() {
  if (!glfwInit()) {
    return;
  }

  glfwSetTime(0);

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  glfwWindowHint(GLFW_SAMPLES, 0);
  glfwWindowHint(GLFW_RED_BITS, 8);
  glfwWindowHint(GLFW_GREEN_BITS, 8);
  glfwWindowHint(GLFW_BLUE_BITS, 8);
  glfwWindowHint(GLFW_ALPHA_BITS, 8);
  glfwWindowHint(GLFW_STENCIL_BITS, 8);
  glfwWindowHint(GLFW_DEPTH_BITS, 24);
  glfwWindowHint(GLFW_RESIZABLE, GL_TRUE);

  // Create a GLFWwindow object
  window = glfwCreateWindow(800, 800, "Cloth Simulator", nullptr, nullptr);
  if (window == nullptr) {
    std::cout << "Failed to create GLFW window" << std::endl;
    glfwTerminate();
    return;
  }
  glfwMakeContextCurrent(window);

  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
    throw std::runtime_error("Could not initialize GLAD!");
  }
  glGetError(); // pull and ignore unhandled errors like GL_INVALID_ENUM

  glClearColor(0.2f, 0.25f, 0.3f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT);

  // Create a nanogui screen and pass the glfw pointer to initialize
  screen = new Screen();
  screen->initialize(window, true);

  int width, height;
  glfwGetFramebufferSize(window, &width, &height);
  glViewport(0, 0, width, height);
  glfwSwapInterval(1);
  glfwSwapBuffers(window);
}

void setGLFWCallbacks() {
  glfwSetCursorPosCallback(window, [](GLFWwindow *, double x, double y) {
    if (!screen->cursorPosCallbackEvent(x, y)) {
      app->cursorPosCallbackEvent(x / screen->pixelRatio(),
                                  y / screen->pixelRatio());
    }
  });

  glfwSetMouseButtonCallback(
      window, [](GLFWwindow *, int button, int action, int modifiers) {
        if (!screen->mouseButtonCallbackEvent(button, action, modifiers) ||
            action == GLFW_RELEASE) {
          app->mouseButtonCallbackEvent(button, action, modifiers);
        }
      });

  glfwSetKeyCallback(
      window, [](GLFWwindow *, int key, int scancode, int action, int mods) {
        if (!screen->keyCallbackEvent(key, scancode, action, mods)) {
          app->keyCallbackEvent(key, scancode, action, mods);
        }
      });

  glfwSetCharCallback(window, [](GLFWwindow *, unsigned int codepoint) {
    screen->charCallbackEvent(codepoint);
  });

  glfwSetDropCallback(window,
                      [](GLFWwindow *, int count, const char **filenames) {
                        screen->dropCallbackEvent(count, filenames);
                        app->dropCallbackEvent(count, filenames);
                      });

  glfwSetScrollCallback(window, [](GLFWwindow *, double x, double y) {
    if (!screen->scrollCallbackEvent(x, y)) {
      app->scrollCallbackEvent(x, y);
    }
  });

  glfwSetFramebufferSizeCallback(window,
                                 [](GLFWwindow *, int width, int height) {
                                   screen->resizeCallbackEvent(width, height);
                                   app->resizeCallbackEvent(width, height);
                                 });
}

void usageError(const char *binaryName) {
  printf("Usage: %s [options]\n", binaryName);
  printf("Required program options:\n");
  printf("  -f     <STRING>    Filename of scene\n");
  printf("  -r     <STRING>    Project root.\n");
  printf("                     Should contain \"shaders/Default.vert\".\n");
  printf("                     Automatically searched for by default.\n");
  printf("  -a     <INT>       Sphere vertices latitude direction.\n");
  printf("  -o     <INT>       Sphere vertices longitude direction.\n");
  printf("  -p     <INT>       Output particle data folder.\n");
  printf("  -i     <INT>       Input particle data folder.\n");
  printf("  -n     <INT>       off for no window.\n");
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
  double particle_mass = object["particle_mass"];
  double density = object["density"];
  double cube_size_per_particle = 1. / pow(density/particle_mass, 1./3.);
  double h = object["h"];
  double epsilon = object["epsilon"];
  double n = object["n"];
  double k = object["k"];
  double c = object["c"];
  auto shape = object["shape"];
  if (!shape.is_array()) {
    throw std::runtime_error("Fluid shape should be an array");
  }

  unique_ptr<vector<Fluid::Triad>> particles = make_unique<vector<Fluid::Triad>>();
  for (const auto &el: shape) {
    string type = el["type"];
    if (type == "cube") {
      vector<double> origin = el["origin"];
      vector<double> size = el["size"];
      for (double i = origin[0] + cube_size_per_particle/2; i < origin[0] + size[0]; i += cube_size_per_particle) {
        for (double j = origin[1] + cube_size_per_particle/2; j < origin[1] + size[1]; j += cube_size_per_particle) {
          for (double k = origin[2] + cube_size_per_particle/2; k < origin[2] + size[2]; k += cube_size_per_particle) {
            particles->emplace_back(Fluid::Triad{i, j, k});
          }
        }
      }
    } else if (type == "sphere") {
      vector<double> origin = el["origin"];
      double radius = el["radius"];
      double r2 = pow(radius, 2);
      for (double i = origin[0]-radius + cube_size_per_particle/2; i < origin[0]+radius; i += cube_size_per_particle) {
        for (double j = origin[1]-radius + cube_size_per_particle/2; j < origin[1]+radius; j += cube_size_per_particle) {
          for (double k = origin[2]-radius + cube_size_per_particle/2; k < origin[2]+radius; k += cube_size_per_particle) {
            if (pow(i-origin[0], 2) + pow(j-origin[1], 2) + pow(k-origin[2], 2)< r2) {
              particles->emplace_back(Fluid::Triad{i, j, k});
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
  fluid = make_shared<Fluid>(std::move(particles), nullptr, h);
  fp = make_shared<FluidParameters>(density, particle_mass, 0.1, h, epsilon, n, k, c);

  object = j[COLLISIONS];
  if (!object.is_array()) {
    throw std::runtime_error(COLLISIONS + std::string(" should be an array"));
  }
  for (const auto &el: object) {
    const string type = el["type"];
    vector<double> vec_point = el["point"];
    vector<double> vec_normal = el["normal"];
    double friction = el["friction"];
    Vector3D point{vec_point[0], vec_point[1], vec_point[2]};
    Vector3D normal{vec_normal[0], vec_normal[1], vec_normal[2]};
    Plane *p = new Plane(point, normal, friction);
    objects->push_back(p);
  }

  object = j[EXTERNAL_FORCES];
  if (object.is_array()) {
    vector<double> external_forces_vec = object;
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
  
  int sphere_num_lat = 3;
  int sphere_num_lon = 3;
  
  std::string file_to_load_from;
  bool file_specified = false;

  std::string particle_foldername_to_input;
  std::string particle_foldername_to_output;
  bool withoutWindow = false;
  
  while ((c = getopt (argc, argv, "f:r:a:o:p:i:n")) != -1) {
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
      case 'a': {
        int arg_int = atoi(optarg);
        if (arg_int < 1) {
          arg_int = 1;
        }
        sphere_num_lat = arg_int;
        break;
      }
      case 'o': {
        int arg_int = atoi(optarg);
        if (arg_int < 1) {
          arg_int = 1;
        }
        sphere_num_lon = arg_int;
        break;
      }
      case 'p': {
        particle_foldername_to_output = optarg;
        break;
      }
      case 'i': {
        particle_foldername_to_input = optarg;
        break;
      }
      case 'n': {
        withoutWindow = true;
        break;
      }
      default: {
        usageError(argv[0]);
        break;
      }
    }
  }
  
  if (!found_project_root) {
    std::cout << "Error: Could not find required file \"shaders/Default.vert\" anywhere!" << std::endl;
    return -1;
  } else {
    std::cout << "Loading files starting from: " << project_root << std::endl;
  }

  #ifdef _OPENMP
    cout << "OpenMP enabled" << endl;
  # endif

  if (!file_specified) { // No arguments, default initialization
    std::stringstream def_fname;
    def_fname << project_root;
    def_fname << "/scene/fluid.json";
    file_to_load_from = def_fname.str();
  }
  
  bool success = loadObjectsFromFile(file_to_load_from, fluid, fp, &objects);
  
  if (!success) {
    std::cout << "Warn: Unable to load from file: " << file_to_load_from << std::endl;
  }

  bool particle_folder_to_output_good = false;
  if (particle_foldername_to_output.length() != 0) {
      int err;
      // from https://stackoverflow.com/questions/23427804/cant-find-mkdir-function-in-dirent-h-for-windows
      #ifdef _WIN32
        err = _mkdir(particle_foldername_to_output.c_str());
      #else 
        err = mkdir(particle_foldername_to_output.c_str(),0755);
      #endif
    particle_folder_to_output_good = err == 0;
    if (!particle_folder_to_output_good) {
      std::cout << "Warn: can't mkdir at " << particle_foldername_to_output << ", will not write to it\n";
    }
  }
  
  bool particle_folder_to_input_good = false;
  if (particle_foldername_to_input.length() != 0) {
    particle_folder_to_input_good = FileUtils::directory_exists(particle_foldername_to_input);
    std::cout << "Replaying file in: " << particle_foldername_to_input << std::endl;
  }

  // Initialize the ClothSimulator object

  app = new ClothSimulator(withoutWindow);
  app->loadFluid(fluid);
  app->loadFluidParameters(fp);
  app->loadCollisionObjects(&objects);

  if (withoutWindow) {
    const auto fps = app->getFps();
    constexpr double duration = 1;
    int n = 0;
    if (particle_folder_to_output_good) {
      const auto output_filename = FileUtils::fluid_filename(particle_foldername_to_output, n);
      ofstream particle_file_to_output(output_filename);
      if (particle_file_to_output) {
        particle_file_to_output << *fluid;
      } else {
        throw std::runtime_error(output_filename + string(" is not good to write!"));
      }
      particle_file_to_output.close();
    }
    n++;
    for (int t = 0; t < duration; t++) {
      for (int f = 0; f < fps; f++) {
        app->simulate();
        // output per frame
        if (particle_folder_to_output_good) {
          const auto output_filename = FileUtils::fluid_filename(particle_foldername_to_output, n);
          ofstream particle_file_to_output(output_filename);
          if (particle_file_to_output) {
            particle_file_to_output << *fluid;
          } else {
            throw std::runtime_error(output_filename + string(" is not good to write!"));
          }
          particle_file_to_output.close();
        }
        n++;
      }
    }
  } else {
    glfwSetErrorCallback(error_callback);

    createGLContexts();

    auto renderer = std::make_shared<OpenGLRenderder>(Misc::SphereMesh(sphere_num_lat, sphere_num_lon));
    app->initWindow(project_root, screen, renderer);

    // Call this after all the widgets have been defined

    screen->setVisible(true);
    screen->performLayout();

    // Attach callbacks to the GLFW window

    setGLFWCallbacks();
    int n = 0;
    int max_n = 0;
    if (particle_folder_to_input_good) {
      std::set<string> tmp;
      FileUtils::list_files_in_directory(particle_foldername_to_input, tmp);
      max_n = tmp.size();
    }
    while (!glfwWindowShouldClose(window)) {
      glfwPollEvents();

      glClearColor(0.25f, 0.25f, 0.25f, 1.0f);
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

      if (!app->isPaused() && particle_folder_to_input_good && n < max_n) {
        const auto input_filename = FileUtils::fluid_filename(particle_foldername_to_input, n);
        ifstream particle_file_to_input(input_filename);
        if (particle_file_to_input) {
          particle_file_to_input >> *fluid;
          n++;
        } else {
          std::cout << "Input file: " << input_filename << " not good to read.\n";
        }
        particle_file_to_input.close();
      }
      app->drawContents(!particle_folder_to_input_good);

      // Draw nanogui
      screen->drawContents();
      screen->drawWidgets();

      glfwSwapBuffers(window);

      if (!app->isAlive()) {
        glfwSetWindowShouldClose(window, 1);
      }
    }
  }



  return 0;
}
