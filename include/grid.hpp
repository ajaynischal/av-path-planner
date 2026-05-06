#pragma once

#include <vector>

namespace avpp {

enum class CellType {
  FREE,
  OBSTACLE,
  START,
  GOAL,
  PATH,
  VISITED
};

class Grid {
 public:
  Grid(int rows, int cols);

  void set_cell(int row, int col, CellType type);
  CellType get_cell(int row, int col) const;

  bool in_bounds(int row, int col) const;
  bool is_passable(int row, int col) const;

  int rows() const;
  int cols() const;

 private:
  int rows_{0};
  int cols_{0};
  std::vector<std::vector<CellType>> cells_;
};

}  // namespace avpp
