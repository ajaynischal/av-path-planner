#include "astar.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <queue>
#include <stdexcept>
#include <unordered_map>

namespace avpp {
namespace {

int flat_index(const Grid& g, int row, int col) { return row * g.cols() + col; }

}  // namespace

AStar::AStar(const Grid& grid, HeuristicType heuristic) : grid_(grid), heuristic_(heuristic) {}

float AStar::heuristic(int r1, int c1, int r2, int c2) const {
  const int dr = std::abs(r1 - r2);
  const int dc = std::abs(c1 - c2);
  switch (heuristic_) {
    case HeuristicType::MANHATTAN:
      return static_cast<float>(dr + dc);
    case HeuristicType::EUCLIDEAN:
      return std::sqrt(static_cast<float>(dr * dr + dc * dc));
    case HeuristicType::DIAGONAL:
      return static_cast<float>(std::max(dr, dc));
  }
  return 0.0f;
}

AStarResult AStar::solve(int start_row, int start_col, int goal_row, int goal_col) const {
  if (!grid_.in_bounds(start_row, start_col) || !grid_.in_bounds(goal_row, goal_col)) {
    throw std::out_of_range("Start/goal out of bounds");
  }
  if (!grid_.is_passable(start_row, start_col) || !grid_.is_passable(goal_row, goal_col)) {
    return AStarResult{};
  }

  AStarResult result{};

  if (start_row == goal_row && start_col == goal_col) {
    result.path_found = true;
    result.path.emplace_back(start_row, start_col);
    result.nodes_explored = 1;
    result.path_length = 0.0f;
    return result;
  }

  Grid search_grid = grid_;  // copy: can mark VISITED without touching original

  std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open;
  std::unordered_map<int, AStarNode> visited_best;  // best-known g_cost

  AStarNode start{};
  start.row = start_row;
  start.col = start_col;
  start.g_cost = 0.0f;
  start.f_cost = heuristic(start_row, start_col, goal_row, goal_col);
  start.parent_row = -1;
  start.parent_col = -1;

  open.push(start);
  visited_best.emplace(flat_index(grid_, start_row, start_col), start);

  constexpr int kMoves[8][2] = {
      {-1, 0}, {1, 0}, {0, -1}, {0, 1}, {-1, -1}, {-1, 1}, {1, -1}, {1, 1},
  };

  while (!open.empty()) {
    const AStarNode current = open.top();
    open.pop();

    const int cur_key = flat_index(grid_, current.row, current.col);
    const auto it_best = visited_best.find(cur_key);
    if (it_best == visited_best.end()) {
      continue;
    }
    if (current.g_cost > it_best->second.g_cost) {
      continue;  // stale queue entry
    }

    ++result.nodes_explored;

    if (!(current.row == start_row && current.col == start_col) &&
        !(current.row == goal_row && current.col == goal_col)) {
      if (search_grid.get_cell(current.row, current.col) == CellType::FREE) {
        search_grid.set_cell(current.row, current.col, CellType::VISITED);
        result.visited.emplace_back(current.row, current.col);
      }
    }

    if (current.row == goal_row && current.col == goal_col) {
      result.path_found = true;

      // Reconstruct path: goal -> start then reverse.
      std::vector<std::pair<int, int>> rev;
      AStarNode n = current;
      while (true) {
        rev.emplace_back(n.row, n.col);
        if (n.parent_row == -1 && n.parent_col == -1) {
          break;
        }
        const int parent_key = flat_index(grid_, n.parent_row, n.parent_col);
        const auto it_parent = visited_best.find(parent_key);
        if (it_parent == visited_best.end()) {
          break;  // should not happen
        }
        n = it_parent->second;
      }
      std::reverse(rev.begin(), rev.end());
      result.path = std::move(rev);

      // Path length.
      float length = 0.0f;
      for (std::size_t i = 1; i < result.path.size(); ++i) {
        const int pr = result.path[i - 1].first;
        const int pc = result.path[i - 1].second;
        const int r = result.path[i].first;
        const int c = result.path[i].second;
        const bool diagonal = (pr != r) && (pc != c);
        length += diagonal ? DIAGONAL_COST : CARDINAL_COST;
      }
      result.path_length = length;
      return result;
    }

    for (const auto& mv : kMoves) {
      const int nr = current.row + mv[0];
      const int nc = current.col + mv[1];
      if (!grid_.in_bounds(nr, nc) || !grid_.is_passable(nr, nc)) {
        continue;
      }

      const bool diagonal = (mv[0] != 0) && (mv[1] != 0);
      const float step_cost = diagonal ? DIAGONAL_COST : CARDINAL_COST;
      const float tentative_g = current.g_cost + step_cost;

      const int nkey = flat_index(grid_, nr, nc);
      const auto it = visited_best.find(nkey);
      if (it != visited_best.end() && tentative_g >= it->second.g_cost) {
        continue;
      }

      AStarNode next{};
      next.row = nr;
      next.col = nc;
      next.g_cost = tentative_g;
      next.f_cost = tentative_g + heuristic(nr, nc, goal_row, goal_col);
      next.parent_row = current.row;
      next.parent_col = current.col;

      visited_best[nkey] = next;
      open.push(next);
    }
  }

  result.path_found = false;
  result.path.clear();
  result.path_length = 0.0f;
  return result;
}

std::string heuristic_name(HeuristicType h) {
  switch (h) {
    case HeuristicType::MANHATTAN:
      return "Manhattan";
    case HeuristicType::EUCLIDEAN:
      return "Euclidean";
    case HeuristicType::DIAGONAL:
      return "Diagonal";
  }
  return "Unknown";
}

}  // namespace avpp
