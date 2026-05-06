#pragma once

#include "grid.hpp"

#include <string>

namespace avpp {

struct Scenario {
  std::string name;
  Grid grid;
  int start_row{0};
  int start_col{0};
  int goal_row{0};
  int goal_col{0};
};

class ScenarioBuilder {
 public:
  static Scenario open_field();
  static Scenario corridor();
  static Scenario maze_like();
};

}  // namespace avpp
