#include "config.hpp"
#include "entity.hpp"
#include "finder.hpp"
#include "reader.hpp"
#include "writer.hpp"

inline void stream_io_init() {
  std::ios::sync_with_stdio(false);
  // std::cin.tie(nullptr);
  // std::cout.tie(nullptr);
}

const char* DIR_NAME[] = {"right", "left", "up", "down"};
const char* BOAT_STATUS[] = {"Idle", "Moving", "Ship", "Go", "Loading"};

/** robot count: 1
 * 1. 22201 -> 26970/25652
 * 2. 24666 -> 25622
 * 3. 23536 -> 26805/25858
 * 4. 10943 -> 13987/13427
 * 5. 16384 -> 20993/19823
 * 6. 28413 -> 29172/29732
 * 7. 17242 -> 18753/18363
 * 8. 20385 -> 22081
 */

int main(int argc, char** argv) {
  stream_io_init();
  GameConfig initial_config;
  EqWeightGrid map;
  Reader reader;
  Writer writer;
  reader.initilize(map, initial_config);

  GameStatus game_status;
  std::deque<Goods> goods;
  // std::unordered_set<GridLocation> goods;

  std::vector<Boat> boats(initial_config.num_boat, map.capacity);
#pragma region variables for robots
  // forward GOODS
  std::vector<std::unordered_map<GridLocation, GridLocation>> robot_came_from(
      initial_config.num_robot);
  std::vector<std::unordered_map<GridLocation, double>> robot_cost_so_far(
      initial_config.num_robot);
  std::vector<int32_t> robot_task(initial_config.num_robot, -1);
  std::vector<std::vector<GridLocation>> to_goods(
      initial_config.num_robot); /* track from goods to robot */
  // forward BERTH
  std::vector<std::unordered_map<GridLocation, GridLocation>> berth_came_from(
      initial_config.num_robot);
  std::vector<std::unordered_map<GridLocation, double>> berth_cost_so_far(
      initial_config.num_robot);
  std::vector<std::vector<GridLocation>> to_berth(
      initial_config.num_robot); /* track from berth to goods */
  std::vector<int32_t> robot_terminal(initial_config.num_robot, -1); /* berth to go while carrying */

  std::vector<int32_t> robot_forward(
      initial_config.num_robot); /* (track_idx)  */
  std::vector<int32_t> robot_backward(initial_config.num_robot);
  std::vector<int32_t*> robot_dir(initial_config.num_robot, nullptr);
  std::vector<std::vector<GridLocation>*> robot_target_ptr(
      initial_config.num_robot, nullptr);
#pragma endregion

  writer.ok();

  const int32_t robot_count_limit =
      argc > 1 ? std::atoi(argv[1]) : initial_config.num_robot;
  logger->log("robot", "robot count for game: ", robot_count_limit);
  for (int32_t frame = 0; frame < initial_config.max_frame; ++frame) {
    // get new robot position from new frame (and it will order the robots)
    reader.update_frame<std::deque, std::vector, std::vector>(
        game_status, goods, map.robots, boats);

    // just use one robot for test
    logger->log("tasker", robot_task);
#pragma region robot pathfinder
    // int32_t robot_id = 0;
    for (int32_t robot_id = 0; robot_id < robot_count_limit; robot_id++) {
      if (robot_task[robot_id] ==
          -1) {  // The robot has not been assigned a task yet.
        robot_task[robot_id] = target_select<Goods, std::deque>(
            map.robots[robot_id], goods,
            [](Goods goods, int32_t pred) -> int32_t {
              return goods.value - pred;
            });  // select one good
        if (robot_task[robot_id] == -1) {
          // no goods here
          logger->log("selector", "nothing to select for robot: ", robot_id);
          // continue;
          break;
        } else {  // U really have a target
          search(map, map.robots[robot_id].pos, goods[robot_task[robot_id]].pos,
                 robot_came_from[robot_id], robot_cost_so_far[robot_id]);
          trace<GridLocation>(robot_came_from[robot_id], to_goods[robot_id],
                              goods[robot_task[robot_id]].pos);
          robot_forward[robot_id] = to_goods[robot_id].size() - 1;
          robot_dir[robot_id] = &robot_forward[robot_id];
          goods.erase(goods.begin() +
                      robot_task[robot_id]);  // remove the assigned goods
          robot_target_ptr[robot_id] = &to_goods[robot_id];
          logger->log("to_goods", "robot_forward: ", robot_forward[robot_id],
                      ", robot: ", robot_id,
                      ", track index(robot->goods): ", *robot_dir[robot_id]);
        }
      }
      // on the way
      logger->log("pre/index", "robot: ", robot_id, ", goods carrying: ", map.robots[robot_id].goods,
                  ", dir_index: ", *robot_dir[robot_id]);

      if (*robot_dir[robot_id] == -1) {
        if (map.robots[robot_id].goods) {
          // already collect goods. on the way to berth
          logger->log("pull", "robot ", robot_id,
                      " pull goods: ", robot_task[robot_id]);
          writer.pull(robot_id);
          // the berth receives one goods
          map.terminals[robot_terminal[robot_id]].receive();
          logger->log("berth/load", "berth: ", robot_terminal[robot_id],
                      " receives a goods, current goods: ",
                      map.terminals[robot_terminal[robot_id]].goods_todo);
          robot_task[robot_id] = -1;
          map.robots[robot_id].goods = false;
          robot_terminal[robot_id] = -1;
          // clear puller track
          berth_came_from[robot_id].clear();
          berth_cost_so_far[robot_id].clear();
          to_berth[robot_id].clear();
          robot_target_ptr[robot_id] = nullptr;
        } else {
          logger->log("get", "robot ", robot_id,
                      " get goods: ", robot_task[robot_id]);
          writer.get(robot_id);
          map.robots[robot_id].goods = true;
          robot_dir[robot_id] = &robot_backward[robot_id];
          // clear getter track
          robot_came_from[robot_id].clear();
          robot_cost_so_far[robot_id].clear();
          to_goods[robot_id].clear();

          robot_terminal[robot_id] = target_select<Berth, std::vector>(
              map.robots[robot_id], map.terminals,
              [](Berth b, int32_t pred) -> int32_t {
                return b.load_speed * 100 - pred;
              });
          search(map, map.robots[robot_id].pos,
                 map.terminals[robot_terminal[robot_id]].pos,
                 berth_came_from[robot_id], berth_cost_so_far[robot_id]);
          trace(berth_came_from[robot_id], to_berth[robot_id],
                map.terminals[robot_terminal[robot_id]].pos);
          robot_backward[robot_id] = to_berth[robot_id].size() - 1;
          robot_target_ptr[robot_id] = &to_berth[robot_id];
          logger->log("robot",
                      "track index(goods->berth): ", *robot_dir[robot_id], ", robot: ", robot_id,
                      ", goods:", map.robots[robot_id].goods);
          logger->log("to_berth", "robot_backward: ", *robot_dir[robot_id]);
        }
      } else {
        GridLocation target =
            (*robot_target_ptr[robot_id])[*robot_dir[robot_id]];
        // GridLocation target = map.robots[test_robot_id].goods?
        // to_berth[test_robot_id][*robot_dir[test_robot_id]] :
        // to_goods[test_robot_id][*robot_dir[test_robot_id]];
        logger->log("dir", "robot: ", robot_id, ", source: ", map.robots[robot_id].pos,
                    " -> target: ", target);
        try {
          int32_t dir =
              SquareGrid::get_dirs_index(map.robots[robot_id].pos, target);
          // logger->log("move", {"right", "left", "up", "down"}[dir]);
          logger->log("move", "robot: ", robot_id, " moves toward ", dir, "/", DIR_NAME[dir]);
          writer.move(robot_id, dir);
          *robot_dir[robot_id] -= 1;
        } catch (
            const std::runtime_error& e) {  // runtime_error for no dir found
          logger->log("move/dir", "robot: ", robot_id,
                      "\n\tdir error: ", e.what(),
                      "\n\ttarget_ptr: ", robot_target_ptr[robot_id],
                      "\n\tto_berth.addr: ", &to_berth[robot_id],
                      "\n\tto_goods.addr: ", &to_goods[robot_id],
                      "\nrobot_task: ", robot_task[robot_id]);
          // robot reset (meaning drop the caught goods)
          robot_task[robot_id] = -1;
          map.robots[robot_id].goods = false;
          robot_terminal[robot_id] = -1;

          robot_dir[robot_id] = nullptr;
          // clear get track
          robot_came_from[robot_id].clear();
          robot_cost_so_far[robot_id].clear();
          to_goods[robot_id].clear();

          // clear puller track
          berth_came_from[robot_id].clear();
          berth_cost_so_far[robot_id].clear();
          to_berth[robot_id].clear();
          robot_target_ptr[robot_id] = nullptr;
        }
      }
      logger->log(
          "post/index", "robot: ", robot_id, ", goods carrying: ", map.robots[robot_id].goods,
          ", dir_index: ",
          (robot_dir[robot_id] != nullptr) ? (*robot_dir[robot_id]) : -1,
          ", target berth: ", robot_terminal[robot_id]);
    }
#pragma endregion
#pragma region ship/berth load and tranport
    // update dock info for berths
    for (int32_t i = 0; i < boats.size(); ++i) {
      // only care the loading boat which don't wait or stay at -1
      if (!(boats[i].dock != -1 && boats[i].status == 1)) continue;
      if (map.terminals[boats[i].dock].docker(i)) continue;
      bool dock_result = map.terminals[boats[i].dock].dock(i);
      if (dock_result) {
        logger->log("dock", "boat: ", i, " docked at berth: ", boats[i].dock);
      } else {
        logger->log("dock", "boat: ", i, " wants to dock at berth: ", boats[i].dock, ", but here lies the boat: ", map.terminals[boats[i].dock].dock_boat_id);
      }
    }

    for (int32_t b = 0; b < map.terminals.size(); ++b) {
      if (map.terminals[b].goods_todo > 0 && map.terminals[b].reservable())
        logger->log("berth/status", "berth: ", b, '[' ,map.terminals[b], ']');
    }

    /**
     * TODO: boats schedule optimization
     * 1. a few frame waiting
     * 2. reverse send inference
    */
    for (int32_t i = 0; i < boats.size(); ++i) {
      int32_t boat_status = get_boat_status(boats[i], i, map.terminals);
      logger->log("boat/status", "boat: ", i, ", status: ", BOAT_STATUS[boat_status], ", dock: ", boats[i].dock);
      bool must_go = false;
      if (boat_status > 1) {
        // must_go check
        int32_t arrive_time = frame + map.terminals[boats[i].dock].transport_time + 1;
        must_go = arrive_time == initial_config.max_frame;
        if (must_go) { /* U should go now */
          boat_status = 3;
          logger->log("go", "force boat: ", i, " to leave at frame: ", frame, ". Arrive at ", arrive_time);
        }
      }
      switch (boat_status) {
        case 1: /* skip */ break;
        case 0: // selector (from VP to berth)
        case 2: {// selector (from berth to berth)
          // goto the berth which has the most goods and no other boat wants to approach
          int32_t gc=0, ti=-1;
          for (int32_t b = 0; b < map.terminals.size(); ++b) {
            if (map.terminals[b].goods_todo > gc && map.terminals[b].reservable()) {
              gc = map.terminals[b].goods_todo;
              ti = b;
            }
          }
          if (ti != -1) {
            logger->log("ship/reserve", "boat: ", i, " wanna berth: ", ti, " to deal with goods count: ", gc, ". Arrive at ", frame + 1 + transport_time(boats[i].dock, ti, map.terminals));
            boats[i].dockit(ti);
            map.terminals[ti].reserve(i);
            writer.ship(i, ti);
          }
        }
          break;
        case 3: {// go
          Berth& worker = map.terminals[boats[i].dock];
          bool get_goods = worker.goods_done > 0;
          bool fully_loaded = boats[i].capacity == worker.goods_done;
          bool goods_clear = worker.goods_todo == 0;
          logger->log("leave/check", "boat: ", i,
                      ", goods loaded: ", worker.goods_done,
                      ", fully_loaded: ", fully_loaded,
                      ", must_go: ", must_go);
          if (must_go || (get_goods && (goods_clear || fully_loaded))) {
                // the boat has loaded some goods (or fully loaded)
                // and there is no remaining goods
            logger->log(
                "go", "boat: ", i, " leaves from berth: ", boats[i].dock,
                " with goods carrying: ", worker.goods_done,
                " it should score at frame: ", frame + 1 + worker.transport_time);
            boats[i].leave();
            worker.leave();
            writer.go(i);
          }
        }
          break;
        case 4: {// loading. call the func
          int32_t berth_id = boats[i].dock;
          bool finish = map.terminals[berth_id].load();
          logger->log("berth/info", "berth: ", berth_id,
                      " goods remaining: ", map.terminals[berth_id].goods_todo,
                      ", goods have been loaded: ", map.terminals[berth_id].goods_done);
        }
          break;
      }
    }

#pragma endregion
    writer.ok();
  }
  writer.ok();  // game over
  logger->log("berth/check", map.terminals);
  return 0;
}