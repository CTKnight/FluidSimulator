#include "iostream"
#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "../clothSimulator.h"
#include "plane.h"

using namespace std;
using namespace CGL;

#define SURFACE_OFFSET 0.0001

void Plane::collide(Vector3R &position, Vector3R &delta_p) {
  const auto &predicted_position = position + delta_p;
  // TODO: assumption: axis aligned
  for (int i = 0; i < 3; i++) {
    const auto diff = predicted_position[i]-point[i];
    if (normal[i] == 1) {
      if (diff < 0) {
        // delta_p[i] = point[i] - diff - position[i];
        delta_p[i] = point[i]- position[i];
      }
      return;
    } else if (normal[i] == -1) {
      if (diff > 0) {
        // delta_p[i] = point[i] - diff - position[i];
        delta_p[i] = point[i]- position[i];
      }
      return;
    }
  }
}
