#include "scorer.h"

#include "config.hpp"
#include "logger.hpp"

const char* BOAT_STATUS[] = {"Idle", "Moving", "Ship", "Go", "Loading"};

using Boats = std::vector<Boat>;
using Berths = std::vector<Berth>;

void dock_update(Boats& boats, Berths& berths) {
  for (int32_t i = 0; i < boats.size(); ++i) {
    // only care the loading boat which don't wait or stay at -1
    if (!(boats[i].dock != -1 && boats[i].status == 1)) continue;
    if (berths[boats[i].dock].docker(i)) continue;
    bool dock_result = berths[boats[i].dock].dock(i);
    if (dock_result) {
      logger->info("dock", "boat: ", i, " docked at berth: ", boats[i].dock);
    } else {
      logger->info(
          "dock", "boat: ", i, " wants to dock at berth: ", boats[i].dock,
          ", but here lies the boat: ", berths[boats[i].dock].dock_boat_id);
    }
  }
}

void greedy_scorer(int32_t frame, Boats& boats, Berths& berths,
                   Writer& writer) {
  // update dock info for berths
  dock_update(boats, berths);

  // basic greedy logic for no boat wait
  for (int32_t b = 0; b < berths.size(); ++b) {
    if (berths[b].goods_todo > 0 && berths[b].reservable())
      logger->info("berth/status", "berth: ", b, '[', berths[b], ']');
  }

  for (int32_t i = 0; i < boats.size(); ++i) {
    int32_t boat_status = get_boat_status(boats[i], i, berths);
    logger->debug("boat/status", "boat: ", i,
                  ", status: ", BOAT_STATUS[boat_status],
                  ", dock: ", boats[i].dock);
    bool must_go = false;
    if (boat_status > 1) {
      // must_go check
      int32_t arrive_time = frame + berths[boats[i].dock].transport_time + 1;
      must_go = arrive_time == GameConfig::max_frame;
      if (must_go) { /* U should go now */
        boat_status = 3;
        logger->info("go", "force boat: ", i, " to leave at frame: ", frame,
                     ". Arrive at ", arrive_time);
      }
    }
    switch (boat_status) {
      case 1: /* skip */
        break;
      case 0:    // selector (from VP to berth)
      case 2: {  // selector (from berth to berth)
        // goto the berth which has the most goods and no other boat wants to
        // approach
        int32_t gc = 0, ti = -1;
        for (int32_t b = 0; b < berths.size(); ++b) {
          if (berths[b].goods_todo > gc && berths[b].reservable()) {
            gc = berths[b].goods_todo;
            ti = b;
          }
        }
        if (ti != -1) {
          logger->info("ship/reserve", "boat: ", i, " wanna berth: ", ti,
                       " to deal with goods count: ", gc, ". Arrive at ",
                       frame + 1 + transport_time(boats[i].dock, ti, berths));
          boats[i].dockit(ti);
          berths[ti].reserve(i);
          writer.ship(i, ti);
        }
      } break;
      case 3: {  // go
        Berth& worker = berths[boats[i].dock];
        bool get_goods = worker.goods_done > 0;
        bool fully_loaded = boats[i].capacity == worker.goods_done;
        bool goods_clear = worker.goods_todo == 0;
        logger->debug("leave/check", "boat: ", i,
                      ", goods loaded: ", worker.goods_done,
                      ", fully_loaded: ", fully_loaded, ", must_go: ", must_go);
        if (must_go || (get_goods && (goods_clear || fully_loaded))) {
          // the boat has loaded some goods (or fully loaded)
          // and there is no remaining goods
          logger->info(
              "go", "boat: ", i, " leaves from berth: ", boats[i].dock,
              " with goods carrying: ", worker.goods_done,
              " it should score at frame: ", frame + 1 + worker.transport_time);
          boats[i].leave();
          worker.leave();
          writer.go(i);
        }
      } break;
      case 4: {  // loading. call the func
        int32_t berth_id = boats[i].dock;
        bool finish = berths[berth_id].load();
        logger->debug(
            "berth/info", "berth: ", berth_id,
            " goods remaining: ", berths[berth_id].goods_todo,
            ", goods have been loaded: ", berths[berth_id].goods_done);
      } break;
    }
  }
}

void lazy_scorer(int32_t frame, Boats& boats, Berths& berths, Writer& writer) {
  // lazy logic is pretty easier than greedy logic as only one bot runs
  // update berth dock info
  dock_update(boats, berths);

  // boat should rush into the one berth
  for (int32_t i = 0; i < berths.size(); ++i) {
    if (berths[i].goods_todo > 0) {  // just send logic
      for (int32_t j = 0; j < boats.size(); ++j) {
        // if (boats[j].waiting()) continue;
        if (!boats[j].idle()) continue;
        // TODO: process send and wait logic
        // send the boats here
        if (boats[j].ready() || berths[i].dock_boat_id == -1) {
          logger->info("ship/reserve", "boat: ", j, " wanna berth: ", i,
                       " to deal with goods count: ", berths[i].goods_todo,
                       ". Arrive at ",
                       frame + 1 + transport_time(boats[j].dock, i, berths));
          boats[j].dockit(i);
          writer.ship(j, i);
        }
      }
    }
    if (berths[i].goods_done > 0) {
      // go & load logic
      for (int32_t j = 0; j < boats.size(); ++j) {
        if (boats[j].status == 0 || boats[j].dock != i) continue;

        Berth& worker = berths[i];
        int32_t arrive_time = frame + berths[i].transport_time + 1;
        bool must_go = arrive_time == GameConfig::max_frame;
        bool fully_loaded = boats[j].capacity == worker.goods_done;
        logger->debug("leave/check", "boat: ", j,
                      ", goods loaded: ", worker.goods_done,
                      ", capacity: ", boats[j].capacity,
                      ", fully_loaded: ", fully_loaded, ", must_go: ", must_go);
        if (must_go || fully_loaded) {
          // the boat has loaded some goods (or fully loaded)
          // and there is no remaining goods
          logger->info(
              "go", "boat: ", j, " leaves from berth: ", i,
              " with goods carrying: ", worker.goods_done,
              " it should score at frame: ", frame + 1 + worker.transport_time);
          boats[j].leave();
          worker.leave();
          writer.go(j);
        } else {
          // loading
          bool log = berths[i].goods_todo > 0;
          berths[i].load();
          if (!log) continue; /* break also works */
          logger->debug("berth/info", "berth: ", i,
                        " goods remaining: ", berths[i].goods_todo,
                        ", goods have been loaded: ", berths[i].goods_done);
        }
      }
    }
  }
}