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

using Point = GridLocation;

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
  int32_t stay_frame;                  /* for resume reference */
  int32_t dist_lock=200, lock_delta=5; /* for quick goods pick */
  int32_t wait_count=0, wait_limit=3; /* for quick goods pick policy update */
  constexpr static int32_t STAY = 20;  /* frames */
  int32_t switch_limit = 20, switch_bar = 10; /* remain for optimization */
  bool running = true;
  bool goods = false;

  Robot() = default;
  Robot(int32_t x, int32_t y) : pos{x, y} {}
  Robot(GridLocation pos_) : pos(pos_) {}

  bool skip(int32_t path_length) {
    return false; // close it for greedy schedule
    if (path_length <= dist_lock) {
      wait_count++;
      if (wait_count == wait_limit) {
        dist_lock -= lock_delta;
        wait_count = 0;
      } return false;
    } else {
      wait_count--;
      if (wait_count == -wait_limit) {
        dist_lock += lock_delta;
        wait_count = 0;
      } return true;
    }
  }
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
  bool docker(int32_t boat_id) const { return dock_boat_id == boat_id; }
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

int32_t transport_time(int32_t src_idx, int32_t dst_idx, const std::vector<Berth>& berths);

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
    return ready() || waiting();
  }
  /* ready to go anywhere from Virtual Point */
  bool ready() const { return dock == -1 && status == 1; }
  bool waiting() const { return dock != -1 && status == 2; }
};

struct GameStatus {
  int32_t frame, gold;
};

struct Goods {
  GridLocation pos;
  int32_t value, birthday, lifetime = 1000 /* frames */;
  bool stub;

  Goods(int32_t x, int32_t y, int32_t value_, int32_t birthday_)
      : pos{x, y}, value(value_), birthday(birthday_), stub(false) {}
  Goods(): pos{0, 0}, value(0), birthday(0), stub(true) {}

  bool operator==(const Goods& other) const {
    return pos == other.pos && value == other.value && birthday == other.birthday;
  }
};

struct Collector {
  int32_t count=0, value=0;

  Collector() = default;
  void collect(const Goods& goods) {
    count++;
    value += goods.value;
  }
  void recollect(const Goods& prev, const Goods& curr) {
    value += curr.value - prev.value;
  }
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
std::ostream& operator<<(std::ostream& out, const Collector&);
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

