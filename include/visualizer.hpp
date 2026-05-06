#pragma once

#include "astar.hpp"
#include "grid.hpp"

#include <string>

namespace avpp {

class Visualizer {
 public:
  static void render(const Grid& grid, const AStarResult& result, const std::string& title);
};

}  // namespace avpp
