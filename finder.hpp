#include <functional>

#include "entity.hpp"
#include "logger.hpp"

inline double manhattan(GridLocation a, GridLocation b) {
  return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

template <typename El, template <typename> typename Iterable>
auto target_select(const Robot& robot, Iterable<El>& elements,
                   std::function<int32_t(El, int32_t)> value_ordering)
    -> decltype(elements.end()) {
  int32_t max_value = -400, pred_dist = 0, cur_value;
  auto elem_ptr = elements.end();
  for (auto begin = elements.begin(); begin != elements.end(); ++begin) {
    pred_dist = manhattan(robot.pos, begin->pos);
    cur_value = value_ordering(*begin, pred_dist);  // value ordering
    if (cur_value > max_value) {
      max_value = cur_value;
      elem_ptr = begin;
    }
  }
  return elem_ptr;
}

template <typename Grid, typename Location>
void search /* indeed a star search*/
    (Grid& grid, Location launch, Location target,
     std::unordered_map<Location, Location>& came_from,
     std::unordered_map<Location, double>& cost_so_far,
     std::function<double(Location a, Location b)> heuristic = manhattan) {
  PriorityQueue<Location, double> frontier;
  frontier.put(launch, 0);

  came_from[launch] = launch;
  cost_so_far[launch] = 0;

  while (!frontier.empty()) {
    Location current = frontier.get();

    if (current == target) {
      break;
    }

    for (Location next : grid.neighbors(current)) {
      double new_cost = cost_so_far[current] + grid.cost(current, next);
      if (cost_so_far.find(next) == cost_so_far.end() ||
          new_cost < cost_so_far[next]) {
        cost_so_far[next] = new_cost;
        double priority = new_cost + heuristic(next, target);
        frontier.put(next, priority);
        came_from[next] =
            current;  // lastly came_from[target] = target.previous
      }
    }
  }
}

template <typename Location>
void trace(std::unordered_map<Location, Location>& came_from,
           std::vector<Location>& track, Location target) {
  // check whether it is full
  track.emplace_back(target);
  Location previous = came_from[target];
  while (previous != came_from[previous]) {
    track.emplace_back(previous);
    previous = came_from[previous];
  }
  logger->debug("trace", track);  // target, target.previous, ..., start.next
  logger->info("trace", "track length: ", track.size());
}

template <typename Grid, typename Location>
int32_t route(
    Grid& grid, Location launch, Location target, std::vector<Location>& track,
    std::function<double(Location a, Location b)> heuristic = manhattan) {
  std::unordered_map<Location, Location> came_from;
  std::unordered_map<Location, double> cost_so_far;
  search(grid, launch, target, came_from, cost_so_far, heuristic);
  trace(came_from, track, target);
  return track.size();
}

template <typename Location, template <typename> typename Sequence>
int32_t find_nearset_from_loc(Location source, Sequence<Berth>& berths) {
  int32_t index, dist, nearsest = 1 << 31;
  for (int32_t idx = 0; idx < berths.size(); ++idx) {
    dist = manhattan(source, berths.pos);
    if (nearsest > dist) {
      nearsest = dist;
      index = idx;
    }
  }
  return index;
}

template <typename Location, template <typename> typename Sequence>
void assign_nearest_k_from_loc(Location source, Sequence<Berth>& berths,
                               int32_t k) {
  if (k < 1 || k > berths.size())
    throw std::runtime_error("K should be less than berths.size");
  int32_t lengths[berths.size()] = {0};
  for (int32_t idx = 0; idx < berths.size(); ++idx) {
    lengths[idx] = manhattan(source, berths.pos);
  }
  // TODO: this function is not fully implemented
}