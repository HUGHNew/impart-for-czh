#include "grid.h"

std::array<GridLocation, 4> SquareGrid::DIRS = {
    /* Right, Left, Up, Down */
    GridLocation{0, 1}, GridLocation{0, -1}, GridLocation{-1, 0},
    GridLocation{1, 0}};

int32_t SquareGrid::get_dirs_index(Point loc) {
  if (std::abs(loc.x) > 1 || std::abs(loc.y) > 1) {
    throw std::runtime_error("Wrong loc for dirs indexing with " + loc);
  }
  int32_t dir_idx = 0;
  for (int32_t idx = 0; idx < 4; ++idx) {
    if (DIRS[idx] == loc) {
      dir_idx = idx;
      break;
    }
  }
  return dir_idx;
}

int32_t SquareGrid::get_dirs_index(Point source, Point target) {
  return get_dirs_index(Point{target.x - source.x, target.y - source.y});
}

inline bool SquareGrid::in_bounds(Point id) const noexcept {
  return 0 <= id.x && id.x < width && 0 <= id.y && id.y < height;
}

inline bool SquareGrid::passable(Point id) const noexcept {
  return walls.find(id) == walls.end();
}

std::vector<Point> SquareGrid::neighbors(
    Point id) const noexcept {
  std::vector<Point> results;

  for (Point dir : DIRS) {
    Point next{id.x + dir.x, id.y + dir.y};
    if (in_bounds(next) && passable(next)) {
      results.emplace_back(next);
    }
  }

  if ((id.x + id.y) % 2 == 0) {
    // make the path straighter
    std::reverse(results.begin(), results.end());
  }

  return results;
}