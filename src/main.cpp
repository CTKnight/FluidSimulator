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

bool loadObjectsFromFile(string filename, shared_ptr<Fluid> &fluid, shared_ptr<FluidParameters> &fp, vector<CollisionObject *>* objects, int sphere_num_lat, int sphere_num_lon) {
  // TODO: debug dummy
  std::cout << "loadObjectsFromFile: Dummy\n";
  auto position = make_unique<vector<Fluid::Triad>>();
  int dimx = 30;
  position->reserve(pow(dimx, 3));
  for (auto i = 0; i < dimx; i++) {
    for (auto j = 0; j < dimx; j++) {
      for (auto k = 0; k < dimx; k++) {
        position->emplace_back(Fluid::Triad{i/100., j/100., k/100.});
      }
    }
  }
  fp = make_shared<FluidParameters>(1,1,1);
  fp->particleRadius = 0.005;
  fluid = make_shared<Fluid>(std::move(position));
  return true;
  // Read JSON from file
  ifstream i(filename);
  if (!i.good()) {
    return false;
  }
  json j;
  i >> j;

  // Loop over objects in scene
  for (json::iterator it = j.begin(); it != j.end(); ++it) {
    string key = it.key();

    // Check that object is valid
    unordered_set<string>::const_iterator query = VALID_KEYS.find(key);
    if (query == VALID_KEYS.end()) {
      cout << "Invalid scene object found: " << key << endl;
      exit(-1);
    }

    // Retrieve object
    json object = it.value();

    // Parse object depending on type (cloth, sphere, or plane)
    if (key == FLUID) {
      // Cloth
      double width, height;
      int num_width_points, num_height_points;
      float thickness;
      e_orientation orientation;
      vector<vector<int>> pinned;

      auto it_width = object.find("width");
      if (it_width != object.end()) {
        width = *it_width;
      } else {
        incompleteObjectError("cloth", "width");
      }

      auto it_height = object.find("height");
      if (it_height != object.end()) {
        height = *it_height;
      } else {
        incompleteObjectError("cloth", "height");
      }

      auto it_num_width_points = object.find("num_width_points");
      if (it_num_width_points != object.end()) {
        num_width_points = *it_num_width_points;
      } else {
        incompleteObjectError("cloth", "num_width_points");
      }

      auto it_num_height_points = object.find("num_height_points");
      if (it_num_height_points != object.end()) {
        num_height_points = *it_num_height_points;
      } else {
        incompleteObjectError("cloth", "num_height_points");
      }

      auto it_thickness = object.find("thickness");
      if (it_thickness != object.end()) {
        thickness = *it_thickness;
      } else {
        incompleteObjectError("cloth", "thickness");
      }

      auto it_orientation = object.find("orientation");
      if (it_orientation != object.end()) {
        orientation = *it_orientation;
      } else {
        incompleteObjectError("cloth", "orientation");
      }

      auto it_pinned = object.find("pinned");
      if (it_pinned != object.end()) {
        vector<json> points = *it_pinned;
        for (auto pt : points) {
          vector<int> point = pt;
          pinned.push_back(point);
        }
      }

      // Cloth parameters
      double damping, density, ks;

      auto it_damping = object.find("damping");
      if (it_damping != object.end()) {
        damping = *it_damping;
      } else {
        incompleteObjectError("cloth", "damping");
      }

      auto it_density = object.find("density");
      if (it_density != object.end()) {
        density = *it_density;
      } else {
        incompleteObjectError("cloth", "density");
      }

      auto it_ks = object.find("ks");
      if (it_ks != object.end()) {
        ks = *it_ks;
      } else {
        incompleteObjectError("cloth", "ks");
      }

      fp->density = density;
      fp->damping = damping;
      fp->ks = ks;
    } else if (key == COLLISIONS) { // PLANE
      Vector3D point, normal;
      double friction = 0.0;

      auto it_point = object.find("point");
      if (it_point != object.end()) {
        vector<double> vec_point = *it_point;
        point = Vector3D(vec_point[0], vec_point[1], vec_point[2]);
      } else {
        incompleteObjectError("plane", "point");
      }

      auto it_normal = object.find("normal");
      if (it_normal != object.end()) {
        vector<double> vec_normal = *it_normal;
        normal = Vector3D(vec_normal[0], vec_normal[1], vec_normal[2]);
      } else {
        incompleteObjectError("plane", "normal");
      }

      auto it_friction = object.find("friction");
      if (it_friction != object.end()) {
        friction = *it_friction;
      } else {
        incompleteObjectError("plane", "friction");
      }

      Plane *p = new Plane(point, normal, friction);
      objects->push_back(p);
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
  
  bool success = loadObjectsFromFile(file_to_load_from, fluid, fp, &objects, sphere_num_lat, sphere_num_lon);
  
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
