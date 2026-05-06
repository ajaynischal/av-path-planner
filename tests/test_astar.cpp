#include "astar.hpp"
#include "scenario.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <unordered_set>

namespace avpp {
namespace {

void require(bool cond, const char* msg) {
  if (!cond) {
    throw std::runtime_error(msg);
  }
}

bool near(float a, float b, float eps = 1e-3f) { return std::fabs(a - b) <= eps; }

}  // namespace
}  // namespace avpp

int main() {
  using namespace avpp;

  // Test 1: 3x3 clear path.
  {
    Grid g(3, 3);
    g.set_cell(0, 0, CellType::START);
    g.set_cell(2, 2, CellType::GOAL);
    AStar solver(g, HeuristicType::MANHATTAN);
    const AStarResult res = solver.solve(0, 0, 2, 2);
    require(res.path_found, "Test 1 failed: expected path_found");
    require(!res.path.empty(), "Test 1 failed: expected non-empty path");
    require(res.path.front() == std::make_pair(0, 0), "Test 1 failed: path must start at start");
    require(res.path.back() == std::make_pair(2, 2), "Test 1 failed: path must end at goal");
  }

  // Test 2: 3x3 fully blocked (no path).
  {
    Grid g(3, 3);
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        g.set_cell(r, c, CellType::OBSTACLE);
      }
    }
    g.set_cell(0, 0, CellType::START);
    g.set_cell(2, 2, CellType::GOAL);
    AStar solver(g, HeuristicType::MANHATTAN);
    const AStarResult res = solver.solve(0, 0, 2, 2);
    require(!res.path_found, "Test 2 failed: expected no path");
  }

  // Test 3: start == goal, path length is 1 (one node).
  {
    Grid g(3, 3);
    g.set_cell(1, 1, CellType::START);
    AStar solver(g, HeuristicType::EUCLIDEAN);
    const AStarResult res = solver.solve(1, 1, 1, 1);
    require(res.path_found, "Test 3 failed: expected path_found");
    require(res.path.size() == 1, "Test 3 failed: expected path of size 1");
    require(res.path.front() == std::make_pair(1, 1), "Test 3 failed: wrong cell");
  }

  // Test 4: all 3 heuristics find same path length on a simple grid.
  {
    Grid g(5, 5);
    g.set_cell(0, 0, CellType::START);
    g.set_cell(4, 4, CellType::GOAL);

    const AStarResult m = AStar(g, HeuristicType::MANHATTAN).solve(0, 0, 4, 4);
    const AStarResult e = AStar(g, HeuristicType::EUCLIDEAN).solve(0, 0, 4, 4);
    const AStarResult d = AStar(g, HeuristicType::DIAGONAL).solve(0, 0, 4, 4);

    require(m.path_found && e.path_found && d.path_found, "Test 4 failed: all should find path");
    require(near(m.path_length, e.path_length), "Test 4 failed: Manhattan vs Euclidean length");
    require(near(m.path_length, d.path_length), "Test 4 failed: Manhattan vs Diagonal length");
  }

  // Test 5: corridor scenario path goes through the gap at (10,16).
  {
    const Scenario s = ScenarioBuilder::corridor();
    const AStarResult res = AStar(s.grid, HeuristicType::DIAGONAL)
                                .solve(s.start_row, s.start_col, s.goal_row, s.goal_col);
    require(res.path_found, "Test 5 failed: expected path in corridor scenario");

    bool through_gap = false;
    for (const auto& [r, c] : res.path) {
      if (r == 10 && c == 16) {
        through_gap = true;
        break;
      }
    }
    require(through_gap, "Test 5 failed: expected path through gap (10,16)");
  }

  std::cout << "All tests passed.\n";
  return 0;
}

