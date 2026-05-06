#pragma once

#include "grid.hpp"

#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace avpp {

enum class HeuristicType { MANHATTAN, EUCLIDEAN, DIAGONAL };

struct AStarNode {
  int row{0};
  int col{0};
  float g_cost{0.0f};
  float f_cost{0.0f};
  int parent_row{-1};
  int parent_col{-1};

  bool operator>(const AStarNode& other) const { return f_cost > other.f_cost; }
};

struct AStarResult {
  bool path_found{false};
  std::vector<std::pair<int, int>> path;  // start to goal, inclusive
  int nodes_explored{0};
  float path_length{0.0f};

  // For visualization: explored cells (excludes final PATH overlay).
  std::vector<std::pair<int, int>> visited;
};

class AStar {
 public:
  AStar(const Grid& grid, HeuristicType heuristic);

  [[nodiscard]] AStarResult solve(int start_row, int start_col, int goal_row, int goal_col) const;

 private:
  float heuristic(int r1, int c1, int r2, int c2) const;

  const Grid& grid_;
  HeuristicType heuristic_;
};

std::string heuristic_name(HeuristicType h);

constexpr float CARDINAL_COST = 1.0f;
constexpr float DIAGONAL_COST = 1.414f;

}  // namespace avpp
