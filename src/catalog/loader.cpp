#include "startracker/catalog/loader.hpp"
#include "startracker/core/logger.hpp"
#include "startracker/core/types.hpp"
#include <cmath>
#include <fstream>
#include <iostream>
#include <unordered_map>
#include <vector>

namespace ST::catalog::loader {
/**
 * @brief Loads binned lookup table (.bin) file and parse the the table,
 * populates the global lookup veriable.
 * @param filename Reference to the .bin file path string.
 * @param lookup Reference to the global lookup
 * veriable.(std::unordered_map<int, std::vector<StarPair>>)
 * @returns bin_load_ok or bin_load_err
 */
ST::LoaderStatus loadBin(const std::string &filename,
                         ST::BinnedStarMap &lookup) {
  std::ifstream in(filename, std::ios::binary);
  if (!in) {
    // std::cout<<"Err LOading bin !!!!!!!"<<std::endl;
    LOG_ERROR << "Error LOading BIN !!!";
    return ST::LoaderStatus::Error;
  }
  lookup.clear();

  size_t mapSize;
  in.read(reinterpret_cast<char *>(&mapSize), sizeof(mapSize));

  for (size_t i = 0; i < mapSize; ++i) {
    int key;
    in.read(reinterpret_cast<char *>(&key), sizeof(key));

    size_t vecSize;
    in.read(reinterpret_cast<char *>(&vecSize), sizeof(vecSize));

    std::vector<StarPair> vec(vecSize);

    for (size_t j = 0; j < vecSize; ++j) {
      size_t len1, len2;

      in.read(reinterpret_cast<char *>(&len1), sizeof(len1));
      vec[j].id1.resize(len1);
      in.read(&vec[j].id1[0], len1);

      in.read(reinterpret_cast<char *>(&len2), sizeof(len2));
      vec[j].id2.resize(len2);
      in.read(&vec[j].id2[0], len2);
    }

    lookup.emplace(key, std::move(vec));
  }
  return ST::LoaderStatus::Ok;
}

ST::LoaderStatus loadcatalog(std::string &realDAT_filenname_CSV,
                             ST::StarProfileMap &rl_DAT_lookup) {
  auto t1 = std::chrono::high_resolution_clock::now();
  std::ifstream file(realDAT_filenname_CSV);
  if (!file.is_open()) {
    std::cout << "Error: Could not open file: " << realDAT_filenname_CSV
              << std::endl;
    return ST::LoaderStatus::Error;
  }

  std::string line;
  if (!std::getline(file, line))
    return ST::LoaderStatus::Error; // skip header

  int total = 0, skipped = 0, added = 0;
  while (std::getline(file, line)) {
    if (line.empty())
      continue;

    std::stringstream ss(line);
    std::string cell;
    std::vector<std::string> row;
    while (std::getline(ss, cell, ','))
      row.push_back(cell);
    total++;

    if (row.size() >= 4) {
      try {
        rl_DAT_lookup[row[0]].ra_rad = (std::stod(row[1]) * M_PI / 180.0);
        rl_DAT_lookup[row[0]].dec_rad = (std::stod(row[2]) * M_PI / 180.0);
        rl_DAT_lookup[row[0]].mag = std::stod(row[3]);
        // std::cout << "Test Bin " << rl_DAT_lookup[row[0]] << "\n";
        added++;

      } catch (const std::invalid_argument &e) {
        skipped++;
        continue;
      }
    }
  }

  auto t2 = std::chrono::high_resolution_clock::now();
  // std::cout << "CSV load: " << std::chrono::duration<double>(t2 - t1).count()
  //           << " sec" << std::endl;
  // std::cout << "Total pairs: " << total << " | Skipped : " << skipped
  //           << " | Kept: " << (added) << std::endl;
  return ST::LoaderStatus::Ok;
}

} // namespace ST::catalog::loader