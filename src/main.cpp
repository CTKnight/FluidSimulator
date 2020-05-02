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
#include "marchingCube.h"
#include "marchingCube.cpp"
#include "main.h"

typedef uint32_t gid_t;

using namespace std;
using namespace nanogui;

using json = nlohmann::json;

#define msg(s) cerr << "[ClothSim] " << s << endl;

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

void testingMarchingCube() {
    // testing marching cube
    cout << "BEGIN TESTING \n";
    Vector3R ugrid = Vector3R(0.01, 0.01, 0.01);
    Vector3R minbox = Vector3R(1.0, 1.0, 0.0);
    Vector3R maxbox = Vector3R(1.5, 1.5, 0.5);
    vector<array<Real, 3>> part;

    string line;
    ifstream myfile ("example.txt");
    if (myfile.is_open())
    {
        while ( getline (myfile,line) )
        {
            vector <string> tokens;
            stringstream check1(line);
            string intermediate;
            while(getline(check1, intermediate, ' '))  tokens.push_back(intermediate);
            array<Real, 3> arr;
            for(int i = 0; i < tokens.size(); i++) {
                arr[i] = stod(tokens[i]);
            }
            part.push_back(arr);
        }
        myfile.close();
    }
    cout << "part size =" << part.size() << endl;
    MarchingCube cube(6000, 1, 0.1, part, ugrid, minbox, maxbox);
    cube.calculateTriangles(0.1, 0, true);
    cube.calculateTriangles(0.1, 500, true);
    cube.calculateTriangles(0.1, 1000, true);
    cube.calculateTriangles(0.1, 1500, true);
    cube.calculateTriangles(0.1, 2000, true);
    cube.calculateTriangles(0.1, 2500, true);
    cube.calculateTriangles(0.1, 3000, true);
    cube.calculateTriangles(0.1, 3500, true);
    cube.calculateTriangles(0.1, 4000, true);
    cube.calculateTriangles(0.1, 4500, true);
    cube.calculateTriangles(0.1, 5000, true);
    cube.calculateTriangles(0.1, 5500, true);
    cube.calculateTriangles(0.1, 5600, true);
    cube.calculateTriangles(0.1, 5700, true);
    cube.calculateTriangles(0.1, 5800, true);
    cube.calculateTriangles(0.1, 5900, true);
    cube.calculateTriangles(0.1, 5980, true);
    cube.calculateTriangles(0.1, 5990, true);
    cube.calculateTriangles(0.1, 6000, true);
    cube.calculateTriangles(0.1, 6010, true);
    cube.calculateTriangles(0.1, 6020, true);
    cube.calculateTriangles(0.1, 6100, true);
    cube.calculateTriangles(0.1, 6200, true);
    cube.calculateTriangles(0.1, 6300, true);
    cube.calculateTriangles(0.1, 6400, true);
    cube.calculateTriangles(0.1, 6500, true);
    cube.calculateTriangles(0.1, 7000, true);
    cube.calculateTriangles(0.1, 7500, true);
    cube.calculateTriangles(0.1, 8000, true);
    cube.calculateTriangles(0.1, 8500, true);
    cube.calculateTriangles(0.1, 9000, true);
    cube.calculateTriangles(0.1, 9500, true);
    cube.calculateTriangles(0.1, 10000, true);
    cube.writeTrianglesIntoObjs("temp");
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
  int sec = 1;
  
  while ((c = getopt (argc, argv, "f:r:a:o:p:i:ns:")) != -1) {
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
      case 's': {
        sec = atoi(optarg);
        if (sec < 1) {
          sec = 1;
        }
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
    exit(-1);
  }

  bool particle_folder_to_output_good = false;
  if (particle_foldername_to_output.length() != 0) {
    int err = mkdir_main(particle_foldername_to_output.c_str());
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

  fluid->init();

  app = new ClothSimulator(withoutWindow);
  app->loadFluid(fluid);
  app->loadFluidParameters(fp);
  app->loadCollisionObjects(&objects);

  if (withoutWindow) {
    const auto fps = app->getFps();
    double duration = sec;
    int n = 0;
    if (particle_folder_to_output_good) {
      writeFluidToFileN(particle_foldername_to_output, n, *fluid);
    }
    n++;
    for (int t = 0; t < duration; t++) {
      for (int f = 0; f < fps; f++) {
        app->simulate();
        // output per frame
        if (particle_folder_to_output_good) {
          writeFluidToFileN(particle_foldername_to_output, n, *fluid);
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
    // testingMarchingCube();
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
