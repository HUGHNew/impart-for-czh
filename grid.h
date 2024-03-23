#pragma once
#include "entity.hpp"
#include <array>

struct Block {
  Point loc;
  bool passable;
};

struct SquareGrid {
  static std::array<Point, 4> DIRS;
  static int32_t get_dirs_index(Point loc);

  static int32_t get_dirs_index(Point source, Point target);

  int32_t width, height;
  std::unordered_set<Point> walls;
  // std::vector<std::vector<>>

  SquareGrid(int32_t width_, int32_t height_)
      : width(width_), height(height_) {}

  inline bool in_bounds(Point id) const noexcept;

  inline bool passable(Point id) const noexcept;

  std::vector<Point> neighbors(Point id) const noexcept;
  constexpr double cost(const Point& from_node,
                        const Point& to_node) const noexcept {
    return 1;
  }
};

struct EqWeightGrid : SquareGrid {
  std::vector<Berth> terminals;  // terminal berths
  std::vector<Robot> robots;
  int32_t capacity;
  EqWeightGrid(int32_t w = 200, int32_t h = 200, int32_t num_bot = 10,
               int32_t num_bth = 10)
      : SquareGrid(w, h) {
    robots.reserve(num_bot);
    terminals.reserve(num_bth);
  }
};