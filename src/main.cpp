#include "astar.hpp"
#include "scenario.hpp"
#include "visualizer.hpp"

#include <iomanip>
#include <iostream>
#include <vector>

namespace avpp {
namespace {

struct Row {
  HeuristicType h;
  int nodes{0};
  float length{0.0f};
};

void run_scenario(const Scenario& s) {
  std::cout << "============================\n";
  std::cout << s.name << "\n";
  std::cout << "============================\n\n";

  const std::vector<HeuristicType> heuristics = {
      HeuristicType::MANHATTAN,
      HeuristicType::EUCLIDEAN,
      HeuristicType::DIAGONAL,
  };

  std::vector<Row> rows;
  rows.reserve(heuristics.size());

  for (const auto h : heuristics) {
    AStar solver(s.grid, h);
    const AStarResult result = solver.solve(s.start_row, s.start_col, s.goal_row, s.goal_col);

    rows.emplace_back(Row{h, result.nodes_explored, result.path_length});

    std::cout << heuristic_name(h) << " | nodes_explored=" << result.nodes_explored
              << " | path_length=" << std::fixed << std::setprecision(2) << result.path_length
              << "\n";

    Visualizer::render(
        s.grid, result, "Heuristic: " + heuristic_name(h) + " (" + s.name + ")");
    std::cout << "\n";
  }

  std::cout << "Scenario: " << s.name << "\n";
  std::cout << "Heuristic    | Nodes Explored | Path Length\n";
  std::cout << "-------------|----------------|------------\n";
  for (const auto& r : rows) {
    std::cout << std::left << std::setw(12) << heuristic_name(r.h) << " | " << std::right
              << std::setw(14) << r.nodes << " | " << std::setw(10) << std::fixed
              << std::setprecision(2) << r.length << "\n";
  }
  std::cout << "\n";
}

}  // namespace
}  // namespace avpp

int main() {
  using namespace avpp;

  const Scenario s1 = ScenarioBuilder::open_field();
  const Scenario s2 = ScenarioBuilder::corridor();
  const Scenario s3 = ScenarioBuilder::maze_like();

  run_scenario(s1);
  run_scenario(s2);
  run_scenario(s3);

  return 0;
}
