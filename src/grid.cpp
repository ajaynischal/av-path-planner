#include "grid.hpp"

#include <stdexcept>

namespace avpp {

Grid::Grid(int rows, int cols) : rows_(rows), cols_(cols) {
  if (rows_ <= 0 || cols_ <= 0) {
    throw std::invalid_argument("Grid dimensions must be positive");
  }
  cells_.assign(static_cast<std::size_t>(rows_),
                std::vector<CellType>(static_cast<std::size_t>(cols_), CellType::FREE));
}

void Grid::set_cell(int row, int col, CellType type) {
  if (!in_bounds(row, col)) {
    throw std::out_of_range("set_cell out of bounds");
  }
  cells_[static_cast<std::size_t>(row)][static_cast<std::size_t>(col)] = type;
}

CellType Grid::get_cell(int row, int col) const {
  if (!in_bounds(row, col)) {
    throw std::out_of_range("get_cell out of bounds");
  }
  return cells_[static_cast<std::size_t>(row)][static_cast<std::size_t>(col)];
}

bool Grid::in_bounds(int row, int col) const {
  return row >= 0 && row < rows_ && col >= 0 && col < cols_;
}

bool Grid::is_passable(int row, int col) const {
  if (!in_bounds(row, col)) {
    return false;
  }
  const CellType t = get_cell(row, col);
  return t == CellType::FREE || t == CellType::START || t == CellType::GOAL;
}

int Grid::rows() const { return rows_; }
int Grid::cols() const { return cols_; }

}  // namespace avpp
