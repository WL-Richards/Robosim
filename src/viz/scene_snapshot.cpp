#include "scene_snapshot.h"

namespace robosim::viz {

transform transform::identity() {
  transform t{};
  t.m[0][0] = 1.0;
  t.m[1][1] = 1.0;
  t.m[2][2] = 1.0;
  t.m[3][3] = 1.0;
  return t;
}

}  // namespace robosim::viz
