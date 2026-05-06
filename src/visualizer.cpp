#include "visualizer.hpp"

#include <iomanip>
#include <iostream>
#include <unordered_set>

namespace avpp {
namespace {

constexpr const char* kReset = "\033[0m";
constexpr const char* kObstacleBg = "\033[100m";
constexpr const char* kGreenBg = "\033[42m";
constexpr const char* kYellowBg = "\033[43m";
constexpr const char* kDim = "\033[2m";

}  // namespace

void Visualizer::render(const Grid& grid, const AStarResult& result, const std::string& title) {
  std::cout << title << "\n";

  Grid g = grid;  // copy for overlay rendering

  // Overlay visited (dim dots) first.
  for (const auto& [r, c] : result.visited) {
    if (!g.in_bounds(r, c)) {
      continue;
    }
    const CellType t = g.get_cell(r, c);
    if (t == CellType::FREE) {
      g.set_cell(r, c, CellType::VISITED);
    }
  }

  // Overlay path (yellow) second so it wins.
  for (const auto& [r, c] : result.path) {
    if (!g.in_bounds(r, c)) {
      continue;
    }
    const CellType t = g.get_cell(r, c);
    if (t == CellType::FREE || t == CellType::VISITED) {
      g.set_cell(r, c, CellType::PATH);
    }
  }

  for (int r = 0; r < g.rows(); ++r) {
    for (int c = 0; c < g.cols(); ++c) {
      const CellType t = g.get_cell(r, c);
      switch (t) {
        case CellType::FREE:
          std::cout << "  ";
          break;
        case CellType::OBSTACLE:
          std::cout << kObstacleBg << "  " << kReset;
          break;
        case CellType::START:
          std::cout << kGreenBg << "S " << kReset;
          break;
        case CellType::GOAL:
          std::cout << kGreenBg << "G " << kReset;
          break;
        case CellType::PATH:
          std::cout << kYellowBg << "* " << kReset;
          break;
        case CellType::VISITED:
          std::cout << kDim << ". " << kReset;
          break;
      }
    }
    std::cout << "\n";
  }

  std::cout << "path_found: " << (result.path_found ? "true" : "false") << "\n";
  std::cout << "nodes_explored: " << result.nodes_explored << "\n";
  std::cout << std::fixed << std::setprecision(2) << "path_length: " << result.path_length << "\n";
}

}  // namespace avpp

