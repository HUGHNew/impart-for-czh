#include "config.hpp"
#include "entity.hpp"
#include "finder.hpp"
#include "reader.hpp"
#include "writer.hpp"
#include "scorer.h"
#include "grid.h"

inline void stream_io_init() {
  std::ios::sync_with_stdio(false);
  // std::cin.tie(nullptr);
  // std::cout.tie(nullptr);
}

const char* DIR_NAME[] = {"right", "left", "up", "down"};

/** robot count: 1
 * 1. 22201 -> 26970/25652|29862
 * 2. 24666 -> 25622/25622|29454
 * 3. 23536 -> 26805/25858|29075
 * 4. 10943 -> 13987/13427|15929
 * 5. 16384 -> 20993/19823|31079
 * 6. 28413 -> 29172/29732|33338
 * 7. 17242 -> 18753/18363|26553
 * 8. 20385 -> 22081/22081|26269
 */

int main(int argc, char** argv) {
  stream_io_init();
  GameConfig initial_config;
  EqWeightGrid map;
  Reader reader;
  Writer writer;
  Collector collector;
  int32_t last_frame = -1, skip_count = 0;
  reader.initilize(map, initial_config);

  GameStatus game_status;
  // std::deque<Goods> goods;
  std::unordered_set<Goods> goods;

  std::vector<Boat> boats(initial_config.num_boat, map.capacity);
#pragma region variables for robots
  // forward GOODS
  std::vector<Goods> robot_task(initial_config.num_robot);
  typedef decltype(goods.end()) GoodsPtr;
  auto GoodsNullPtr = GoodsPtr(nullptr);
  std::vector<GoodsPtr> robot_task_ptr(initial_config.num_robot, GoodsPtr(GoodsNullPtr));

  std::vector<std::vector<GridLocation>> to_goods(
      initial_config.num_robot); /* track from goods to robot */
  // forward BERTH
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
  logger->warn("robot", "robot count for game: ", robot_count_limit);
  for (int32_t frame = 0; frame < initial_config.max_frame; ++frame) {
    // get new robot position from new frame (and it will order the robots)
    reader.update_frame<std::unordered_set, std::vector, std::vector>(
        game_status, goods, map.robots, boats);
    if (game_status.frame != last_frame+1) {
      logger->error("frame", "last_frame: ", last_frame, ", current_frame: ", game_status.frame, ", skiped: ", game_status.frame - last_frame - 1);
      skip_count++;
    }
    last_frame = game_status.frame;

    // just use one robot for test
    logger->info("tasker", robot_task);
#pragma region robot pathfinder
    // int32_t robot_id = 0;
    for (int32_t robot_id = 0; robot_id < robot_count_limit; robot_id++) {
      if (robot_task_ptr[robot_id] == GoodsNullPtr) {
        // The robot has not been assigned a task yet.
        robot_task_ptr[robot_id] = target_select<Goods, std::unordered_set>(
            map.robots[robot_id], goods,
            [](Goods goods, int32_t pred) -> int32_t {
              return goods.value - pred;
            });  // select one good
        if (robot_task_ptr[robot_id] == GoodsNullPtr) {
          // no goods here
          logger->info("selector", "nothing to select for robot: ", robot_id);
          continue;
          // break;
        } else {  // U really have a target
          robot_task[robot_id] = *(robot_task_ptr[robot_id]);
          int32_t pathlen = route(map, map.robots[robot_id].pos, robot_task[robot_id].pos, to_goods[robot_id]);
          if (map.robots[robot_id].skip(pathlen)) {
            to_goods[robot_id].clear();
            logger->info("selector", "robot: ", robot_id, ", pathlen: ", pathlen, " exceeds desired distance: ", map.robots[robot_id].dist_lock);
            continue;
          }
          logger->info("collection", "collect goods: ", robot_task[robot_id]);
          collector.collect(robot_task[robot_id]);
          robot_forward[robot_id] = pathlen - 1;
          robot_dir[robot_id] = &robot_forward[robot_id];
          goods.erase(robot_task_ptr[robot_id]);
          robot_target_ptr[robot_id] = &to_goods[robot_id];
          logger->info("to_goods", "robot_forward: ", robot_forward[robot_id],
                      ", robot: ", robot_id,
                      ", track index(robot->goods): ", *robot_dir[robot_id]);
        }
      }
      // on the way
      logger->debug("pre/index", "robot: ", robot_id, ", goods carrying: ", map.robots[robot_id].goods,
                  ", dir_index: ", *robot_dir[robot_id]);

      if (*robot_dir[robot_id] == -1) {
        if (map.robots[robot_id].goods) {
          // already collect goods. on the way to berth
          logger->info("pull", "robot ", robot_id,
                      " pull goods: ", robot_task[robot_id]);
          writer.pull(robot_id);
          // the berth receives one goods
          map.terminals[robot_terminal[robot_id]].receive();
          logger->info("berth/load", "berth: ", robot_terminal[robot_id],
                      " receives a goods, current goods: ",
                      map.terminals[robot_terminal[robot_id]].goods_todo);
          robot_task_ptr[robot_id] = GoodsNullPtr;
          map.robots[robot_id].goods = false;
          robot_terminal[robot_id] = -1;
          // clear puller track
          to_berth[robot_id].clear();
          robot_target_ptr[robot_id] = nullptr;
        } else {
          logger->info("get", "robot ", robot_id,
                      " get goods: ", robot_task[robot_id]);
          writer.get(robot_id);
          map.robots[robot_id].goods = true;
          robot_dir[robot_id] = &robot_backward[robot_id];
          // clear getter track
          to_goods[robot_id].clear();

          auto it = target_select<Berth, std::vector>(
              map.robots[robot_id], map.terminals,
              [](Berth b, int32_t pred) -> int32_t {
                return b.load_speed * 100 - pred;
              });
          int32_t idx = it == map.terminals.end() ? -1 : it - map.terminals.begin();
          robot_terminal[robot_id] = idx;
          // robot_terminal[robot_id] = it == map.terminals.end() ? -1 : it - map.terminals.begin();

          route(map, map.robots[robot_id].pos, map.terminals[robot_terminal[robot_id]].pos, to_berth[robot_id]);
          robot_backward[robot_id] = to_berth[robot_id].size() - 1;
          robot_target_ptr[robot_id] = &to_berth[robot_id];
          logger->debug("robot",
                      "track index(goods->berth): ", *robot_dir[robot_id], ", robot: ", robot_id,
                      ", goods:", map.robots[robot_id].goods);
          logger->debug("to_berth", "robot_backward: ", *robot_dir[robot_id]);
        }
      } else {
        GridLocation target =
            (*robot_target_ptr[robot_id])[*robot_dir[robot_id]];
        // GridLocation target = map.robots[test_robot_id].goods?
        // to_berth[test_robot_id][*robot_dir[test_robot_id]] :
        // to_goods[test_robot_id][*robot_dir[test_robot_id]];
        logger->debug("dir", "robot: ", robot_id, ", source: ", map.robots[robot_id].pos,
                    " -> target: ", target);
        try {
          int32_t dir =
              SquareGrid::get_dirs_index(map.robots[robot_id].pos, target);
          logger->debug("move", "robot: ", robot_id, " moves toward ", dir, "/", DIR_NAME[dir]);
          writer.move(robot_id, dir);
          *robot_dir[robot_id] -= 1;
        } catch (
            const std::runtime_error& e) {  // runtime_error for no dir found
          logger->debug("move/dir", "robot: ", robot_id,
                      "\n\tdir error: ", e.what(),
                      "\n\ttarget_ptr: ", robot_target_ptr[robot_id],
                      "\n\tto_berth.addr: ", &to_berth[robot_id],
                      "\n\tto_goods.addr: ", &to_goods[robot_id],
                      "\nrobot_task: ", robot_task[robot_id]);
          // robot reset (meaning drop the caught goods)
          robot_task_ptr[robot_id] = GoodsNullPtr;
          map.robots[robot_id].goods = false;
          robot_terminal[robot_id] = -1;

          robot_dir[robot_id] = nullptr;
          // clear get track
          to_goods[robot_id].clear();

          // clear puller track
          to_berth[robot_id].clear();
          robot_target_ptr[robot_id] = nullptr;
        }
      }
      logger->debug(
          "post/index", "robot: ", robot_id, ", goods carrying: ", map.robots[robot_id].goods,
          ", dir_index: ",
          (robot_dir[robot_id] != nullptr) ? (*robot_dir[robot_id]) : -1,
          ", target berth: ", robot_terminal[robot_id]);
    }
#pragma endregion
#pragma region ship/berth load and tranport
    // update the boats and berths
    greedy_scorer(frame, boats, map.terminals, writer);
    // lazy_scorer(frame, boats, map.terminals, writer);

#pragma endregion
    writer.ok();
  }
  logger->info("collection", collector);
  // logger->debug("berth/check", map.terminals);
  writer.ok();  // game over
  return 0;
}