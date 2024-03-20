#pragma once

#include <algorithm>
#include <array>
#include <deque>
#include <iomanip>
#include <iostream>
#include <queue>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

struct GridLocation {
  int32_t x, y;
  inline operator std::string() const {
    return "(" + std::to_string(x) + "," + std::to_string(y) + ")";
  }
};

#pragma region GridLocation operators
static inline bool operator==(GridLocation lhs, GridLocation rhs) {
  return lhs.x == rhs.x && lhs.y == rhs.y;
}

static inline bool operator!=(GridLocation lhs, GridLocation rhs) {
  return !(lhs == rhs);
}

static inline bool operator<(GridLocation lhs, GridLocation rhs) {
  return std::tie(lhs.x, lhs.y) <
         std::tie(rhs.x, rhs.y);  // just a definition for sorting
}

static inline GridLocation operator+(GridLocation lhs, GridLocation rhs) {
  return GridLocation{lhs.x + rhs.x, lhs.y + rhs.y};
}

static inline std::string operator+(const char* lhs, GridLocation rhs) {
  return lhs + std::string(rhs);
}

static inline std::string operator+(GridLocation lhs, const char* rhs) {
  return std::string(lhs) + rhs;
}
#pragma endregion

namespace std {
/* implement hash function so we can put GridLocation into an unordered_set */
template <>
struct hash<GridLocation> {
  std::size_t operator()(const GridLocation& id) const noexcept {
    // x,y in [0,199]
    return std::hash<int32_t>()(id.x ^ (id.y << 16));
  }
};
}  // namespace std

#pragma region basic game entity

struct Robot {
  GridLocation pos;
  int32_t stay_frame;                 /* for resume reference */
  constexpr static int32_t STAY = 20; /* frames */
  bool running = true;
  bool goods = false;

  Robot() = default;
  Robot(int32_t x, int32_t y) : pos{x, y} {}
  Robot(GridLocation pos_) : pos(pos_) {}
};

struct Berth {
  constexpr static GridLocation size{4, 4};
  GridLocation pos;
  int32_t transport_time, load_speed;
  int32_t goods_todo = 0, goods_done = 0;
  int32_t dock_boat_id = -1, book_boat_id = -1;
  /* What's the sequence of goods loading? */
  /* Impossible to calculate the goods' value on the berth */

  Berth() = default;
  Berth(int32_t x, int32_t y, int32_t trans_time, int32_t load_time)
      : pos{x, y}, transport_time(trans_time), load_speed(load_time) {}
  bool dock(int32_t boat_id) {
    if (dock_boat_id != -1) return false;
    dock_boat_id = boat_id;
    book_boat_id = -1;
    return true;
  }
  bool reservable(){ return dock_boat_id == -1 && book_boat_id == -1; }
  /* no reservable check here */
  void reserve(int32_t boat_id){ book_boat_id = boat_id; }
  void leave() {
    goods_done = 0;
    dock_boat_id = -1;
  }
  bool load() {
    if (goods_todo > 0) {
      if (load_speed > goods_todo) {
        goods_done += goods_todo;
        goods_todo = 0;
      } else {
        goods_done += load_speed;
        goods_todo -= load_speed;
        return false;
      }
    }
    return true;  // clear all goods
  }
  void receive() { goods_todo += 1; }
};
constexpr int32_t transport_time(const Berth& from, const Berth& to) {
  return 500; /* frames */
}

struct Boat {
  constexpr static GridLocation size{2, 4};
  int32_t capacity, status /* 0: move, 1: load, 2: wait */, dock = -1;
  bool leaving = false;

  Boat() = default;
  Boat(int32_t capacity_) : capacity(capacity_) {}
  void dockit(int32_t id) {
    dock = id;
    leaving = false;
  }
  void leave() {
    dock = -1;
    leaving = true;
  }
  /* in the virtual point or wait at the berth */
  bool idle() const { 
    return (dock == -1 && status == 1) || (dock != -1 && status == 2);
  }
};

struct GameStatus {
  int32_t frame, gold;
};

struct Goods {
  GridLocation pos;
  int32_t value, birthday, lifetime = 1000 /* frames */;

  Goods(int32_t x, int32_t y, int32_t value_, int32_t birthday_)
      : pos{x, y}, value(value_), birthday(birthday_) {}
};
namespace std {
/* implement hash function so we can put GridLocation into an unordered_set */
template <>
struct hash<Goods> {
  std::size_t operator()(const Goods& id) const noexcept {
    // the pos is unique
    return std::hash<int32_t>()(id.pos.x ^ (id.pos.y << 16));
  }
};
}  // namespace std
#pragma region operator<< overload for Entities
std::ostream& operator<<(std::ostream& out, GridLocation loc);
std::ostream& operator<<(std::ostream& out, const Robot& robot);
std::ostream& operator<<(std::ostream& out, const Berth& berth);
std::ostream& operator<<(std::ostream& out, const Boat& boat);
std::ostream& operator<<(std::ostream& out, const Goods& goods);
std::ostream& operator<<(std::ostream& out, const GameStatus& status);

template <typename _Tp, template <typename> typename _Seq>
std::ostream& redirect_helper(std::ostream& out, const _Seq<_Tp>& sequence) {
  out << '[';
  auto begin = sequence.begin(), end = sequence.end();
  if (begin != end) {
    out << *begin++;
  }
  while (begin != end) {
    out << ", " << *begin++;
  }
  out << ']';
  return out;
}
// vector
template <typename _Tp>
std::ostream& operator<<(std::ostream& out, const std::vector<_Tp>& sequence) {
  return redirect_helper(out, sequence);
}
// deque
template <typename _Tp>
std::ostream& operator<<(std::ostream& out, const std::deque<_Tp>& sequence) {
  return redirect_helper(out, sequence);
}
#pragma endregion  // operator<<
#pragma endregion

#pragma region entity interaction function
/**
 * 0: idle boat. Able to ship to any berth [boat.idle()]
 * 1: moving. DO NOT DISTURB IT [status==0]
 * loading.[dock!=-1, status==1]
 * 2: nothing to do with no goods. Ship to other berth
 * 3: nothing to do with goods loaded. GO
 * 4: loading. DO NOT DISTURB IT
*/
int32_t get_boat_status(const Boat& boat, int32_t boat_id, const std::vector<Berth>& berths);
#pragma endregion

struct SquareGrid {
  static std::array<GridLocation, 4> DIRS;
  static int32_t get_dirs_index(GridLocation loc) {
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

  static int32_t get_dirs_index(GridLocation source, GridLocation target) {
    return get_dirs_index(
        GridLocation{target.x - source.x, target.y - source.y});
  }

  int32_t width, height;
  std::unordered_set<GridLocation> walls;

  SquareGrid(int32_t width_, int32_t height_)
      : width(width_), height(height_) {}

  inline bool in_bounds(GridLocation id) const noexcept {
    return 0 <= id.x && id.x < width && 0 <= id.y && id.y < height;
  }

  inline bool passable(GridLocation id) const noexcept {
    return walls.find(id) == walls.end();
  }

  std::vector<GridLocation> neighbors(GridLocation id) const noexcept;
};

template <typename Grid>
void parse_surface_from_line(Grid& grid, const std::string& line, int lineno) {
  for (int32_t i = 0; i < line.size(); ++i) {
    switch (line[i]) {
      case '*':
      case '#':
        grid.walls.insert(GridLocation{lineno, i});
        break;
      case 'A':
        grid.robots.emplace_back(Robot(lineno, i));
        break;
    }
  }
}

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
  constexpr double cost(const GridLocation& from_node,
                        const GridLocation& to_node) const noexcept {
    return 1;
  }
};

template <typename T, typename priority_t>
struct PriorityQueue {
  typedef std::pair<priority_t, T> PQElement;
  std::priority_queue<PQElement, std::vector<PQElement>,
                      std::greater<PQElement>>
      elements;  // min-heap

  inline bool empty() const { return elements.empty(); }

  inline void put(T item, priority_t priority) {
    elements.emplace(priority, item);
  }

  T get() {
    T best_item = elements.top().second;
    elements.pop();
    return best_item;
  }
};

struct Game {
  EqWeightGrid map;
  std::deque<Goods> goods;
  GameStatus status;

  Game() = default;
  void test() {
    // goods.erase
  }
};
