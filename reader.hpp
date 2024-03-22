#pragma once

#include "config.hpp"
#include "entity.hpp"
#include "logger.hpp"

struct Reader {
  void skip_line(std::istream& ins) {
    static std::string line;
    ins >> line;
    // ins.ignore(10, '\n');
  }
#pragma region initilization components
  template <typename Grid>
  void read_grid(Grid& grid, const GameConfig& config, std::istream& ins) {
    std::string line;
    for (int32_t i = 0; i < grid.height; ++i) {
      ins >> line;
      parse_surface_from_line<Grid>(grid, line, i);
    }
  }
  template <typename Bt>
  Bt read_berth(std::istream& ins) {
    int32_t bid;
    Bt berth;
    ins >> bid;
    ins >> berth.pos.x >> berth.pos.y >> berth.transport_time >>
        berth.load_speed;
    return berth;
  }
  template <typename Bt>
  void read_berths(std::vector<Bt>& berths, std::istream& ins) {
    logger->debug("reader", "loading berths");
    for (int32_t i = 0; i < berths.capacity(); ++i) {
      const Bt& berth = read_berth<Bt>(ins);
      logger->debug("reader/berth", berth);
      berths.emplace_back(berth);
    }
  }
  void read_capacity(int32_t& capacity, std::istream& ins) { ins >> capacity; }
#pragma endregion

  /**
   * @brief initilize the game environment (map, berths, cargo capacity)
   * @param grid game map
   * @param berths all the terminals
   * @param capacity the capacity of each boat
   * @param config the basic game configuration
   * @param ins the input stream to read from (default to std::cin)
   */
  template <typename Grid, typename Bt>
  void initilize(Grid& grid, std::vector<Bt>& berths, int32_t& capacity,
                 const GameConfig& config, std::istream& ins = std::cin) {
    logger->info("reader", "Reader::initilize");
    read_grid(grid, config, ins);
    read_berths(berths, ins);
    read_capacity(capacity, ins);
    skip_line(ins);  // just ignore "OK"
  }
  template <typename Grid>
  void initilize(Grid& grid, const GameConfig& config,
                 std::istream& ins = std::cin) {
    initilize(grid, grid.terminals, grid.capacity, config, ins);
  }

  // void initilize(EqWeightGrid<10U>& grid, const GameConfig& config,
  // std::istream& ins=std::cin);
#pragma region frame updating components
  void update_game_status(GameStatus& status, std::istream& ins = std::cin) {
    ins >> status.frame >> status.gold;
    logger->info("reader/game_status", status);
  }

  template <template <typename> typename Sequence>
  void update_goods(Sequence<Goods>& goods, int32_t frame,
                    std::istream& ins = std::cin) {
    int32_t num_goods;
    ins >> num_goods;
    logger->info("reader/num_goods", num_goods);
    int32_t x, y, val;
    for (int32_t i = 0; i < num_goods; ++i) {
      ins >> x >> y >> val;
      goods.emplace_back(x, y, val, frame);
      logger->debug("reader/goods", goods.back());
    }
  }

  template <template <typename> typename Sequence>
  void update_robots_status(Sequence<Robot>& robots,
                            std::istream& ins = std::cin) {
    for (Robot& robot : robots) {
      ins >> robot.goods >> robot.pos.x >> robot.pos.y >> robot.running;
      logger->debug("reader/robot", robot);
    }
  }

  template <template <typename> typename Sequence>
  void update_boats_status(Sequence<Boat>& cargo,
                           std::istream& ins = std::cin) {
    for (Boat& boat : cargo) {
      ins >> boat.status >> boat.dock;
      logger->debug("reader/boat", boat);
    }
  }

  template <template <typename> typename SeqGood,
            template <typename> typename SeqBot,
            template <typename> typename SeqBoat>
  void update_frame(GameStatus& status, SeqGood<Goods>& goods,
                    SeqBot<Robot>& robots, SeqBoat<Boat>& cargo,
                    std::istream& ins = std::cin) {
    update_game_status(status, ins);
    update_goods(goods, status.frame, ins);
    update_robots_status(robots, ins);
    update_boats_status(cargo, ins);
    skip_line(ins);  // ignore "OK"
  }
#pragma endregion
};
