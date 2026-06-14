#pragma once
#include "startracker/core/types.hpp"
#include <string>
#include <unordered_map>
#include <vector>

namespace ST::catalog::loader {

ST::LoaderStatus loadBin(const std::string &filename,
                         ST::BinnedStarMap &lookup);

ST::LoaderStatus loadcatalog(std::string &realDAT_filenname_CSV,
                             ST::StarProfileMap &rl_DAT_lookup);

} // namespace ST::catalog::loader