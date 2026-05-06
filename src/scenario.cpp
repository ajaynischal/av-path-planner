#include "scenario.hpp"

#include <random>
#include <utility>
#include <vector>

namespace avpp {
namespace {

constexpr int kSize = 20;

void mark_start_goal(Grid& g, int sr, int sc, int gr, int gc) {
  g.set_cell(sr, sc, CellType::START);
  g.set_cell(gr, gc, CellType::GOAL);
}

}  // namespace

Scenario ScenarioBuilder::open_field() {
  Scenario s{
      .name = "Open Field",
      .grid = Grid(kSize, kSize),
      .start_row = 0,
      .start_col = 0,
      .goal_row = kSize - 1,
      .goal_col = kSize - 1,
  };

  std::mt19937 rng(42);  // deterministic "random" for reproducible portfolio output
  std::bernoulli_distribution obstacle(0.15);

  for (int r = 0; r < s.grid.rows(); ++r) {
    for (int c = 0; c < s.grid.cols(); ++c) {
      if ((r == s.start_row && c == s.start_col) || (r == s.goal_row && c == s.goal_col)) {
        continue;
      }
      if (obstacle(rng)) {
        s.grid.set_cell(r, c, CellType::OBSTACLE);
      }
    }
  }

  mark_start_goal(s.grid, s.start_row, s.start_col, s.goal_row, s.goal_col);
  return s;
}

Scenario ScenarioBuilder::corridor() {
  Scenario s{
      .name = "Corridor",
      .grid = Grid(kSize, kSize),
      .start_row = 0,
      .start_col = 0,
      .goal_row = kSize - 1,
      .goal_col = kSize - 1,
  };

  const int wall_row = 10;
  for (int c = 0; c <= 15; ++c) {
    s.grid.set_cell(wall_row, c, CellType::OBSTACLE);
  }
  for (int c = 17; c <= 19; ++c) {
    s.grid.set_cell(wall_row, c, CellType::OBSTACLE);
  }
  // Gap at (10,16) left FREE.

  mark_start_goal(s.grid, s.start_row, s.start_col, s.goal_row, s.goal_col);
  return s;
}

Scenario ScenarioBuilder::maze_like() {
  Scenario s{
      .name = "Maze-like",
      .grid = Grid(kSize, kSize),
      .start_row = 0,
      .start_col = 0,
      .goal_row = kSize - 1,
      .goal_col = kSize - 1,
  };

  const std::vector<std::pair<int, int>> obstacles = {
      {1, 3},  {1, 4},  {1, 5},  {2, 5},  {3, 5},  {4, 5},  {5, 5},
      {6, 2},  {6, 3},  {6, 4},  {7, 4},  {8, 4},  {9, 4},
      {3, 10}, {4, 10}, {5, 10}, {6, 10}, {7, 10}, {8, 10},
      {10, 7}, {10, 8}, {10, 9}, {10, 10}, {10, 11}, {10, 12},
      {12, 2}, {13, 2}, {14, 2}, {15, 2}, {16, 2},
      {12, 14}, {13, 14}, {14, 14}, {15, 14}, {16, 14},
      {17, 6}, {17, 7}, {17, 8}, {16, 8}, {15, 8},
      {8, 15}, {8, 16}, {8, 17}, {9, 17}, {10, 17}, {11, 17},
  };

  for (const auto& [r, c] : obstacles) {
    if ((r == s.start_row && c == s.start_col) || (r == s.goal_row && c == s.goal_col)) {
      continue;
    }
    if (s.grid.in_bounds(r, c)) {
      s.grid.set_cell(r, c, CellType::OBSTACLE);
    }
  }

  mark_start_goal(s.grid, s.start_row, s.start_col, s.goal_row, s.goal_col);
  return s;
}

}  // namespace avpp
