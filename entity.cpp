#include "entity.hpp"

std::vector<GridLocation> SquareGrid::neighbors(
    GridLocation id) const noexcept {
  std::vector<GridLocation> results;

  for (GridLocation dir : DIRS) {
    GridLocation next{id.x + dir.x, id.y + dir.y};
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

int32_t transport_time(int32_t src_idx, int32_t dst_idx, const std::vector<Berth>& berths) {
  if (src_idx == -1 || dst_idx == -1) { // They can't both be -1 (VP)
    return berths[src_idx+dst_idx+1].transport_time;
  } else {
    return 500; /* frames */
  }
}

std::array<GridLocation, 4> SquareGrid::DIRS = {
    /* Right, Left, Up, Down */
    GridLocation{0, 1}, GridLocation{0, -1}, GridLocation{-1, 0},
    GridLocation{1, 0}};

#pragma region operator<< implementation
std::ostream& operator<<(std::ostream& out, GridLocation loc) {
  return out << '(' << loc.x << ',' << loc.y << ')';
}

std::ostream& operator<<(std::ostream& out, const Robot& robot) {
  return out << robot.pos << ", goods: " << robot.goods;
}

std::ostream& operator<<(std::ostream& out, const Berth& berth) {
  return out << berth.pos << ", trans: " << berth.transport_time
             << ", speed: " << berth.load_speed
             << ", goods: " << berth.goods_todo << "/" << berth.goods_done
             << ", boat_id: " << berth.dock_boat_id
             << ", book_boat_id: " << berth.book_boat_id;
}

std::ostream& operator<<(std::ostream& out, const Boat& boat) {
  return out << "capacity: " << boat.capacity << ", status: " << boat.status
             << ", dock: " << boat.dock;
}

std::ostream& operator<<(std::ostream& out, const Goods& goods) {
  return out << goods.pos << ", value: " << goods.value
             << ", birthday: " << goods.birthday;
}

std::ostream& operator<<(std::ostream& out, const Collector& collector) {
  return out << "count: " << collector.count << ", value: " << collector.value;
}

std::ostream& operator<<(std::ostream& out, const GameStatus& status) {
  return out << "frame: " << status.frame << ", gold: " << status.gold;
}
#pragma endregion


#pragma region entity interaction function
int32_t get_boat_status(const Boat& boat, int32_t boat_id, const std::vector<Berth>& berths) {
  if (boat.idle()) { return 0; }
  if (boat.status == 0) { return 1; }
  if (boat.status != 1) { throw std::runtime_error("boat.status must be 1"); }
  if (berths[boat.dock].goods_todo == 0) {
    if (berths[boat.dock].goods_done == 0) {
      return 2;
    } else return 3;
  } else {
    if (berths[boat.dock].goods_done == boat.capacity) {
      return 3;
    }
  }

  return 4; // loading
}
#pragma endregion