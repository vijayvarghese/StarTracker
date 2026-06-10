#pragma once
#include "startracker/core/types.hpp"
#include <string>
#include <unordered_map>
#include <vector>

namespace ST::catalog {

BinLoadStatus loadBin(const std::string &filename,
                      std::unordered_map<int, std::vector<StarPair>> &lookup);

} // namespace ST::catalog