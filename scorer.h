#pragma once

#include "entity.hpp"
#include "writer.hpp"

void greedy_scorer(int32_t frame, std::vector<Boat>& boats,
                   std::vector<Berth>& berths, Writer& writer);
void lazy_scorer(int32_t frame, std::vector<Boat>& boats,
                 std::vector<Berth>& berths, Writer& writer);