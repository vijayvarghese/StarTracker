#pragma once
#include<unordered_map>
#include<vector>
#include<string>
#include "startracker/core/types.hpp"


namespace ST::catalog
{

BinLoadStatus loadBin(const std::string& filename,
                std::unordered_map<int, std::vector<StarPair>>& lookup);

} // namespace ST::catalog::binload