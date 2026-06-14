#include "startracker/attitude/quest.hpp"
#include "startracker/core/types.hpp"
#include <iomanip>
#include <iostream>
#include <string>
#include <unordered_map>

void quest(const std::string &HipID, const ST::StarProfileMap &rl_DAT_lookup) {
  const auto &star = rl_DAT_lookup.at(HipID);
  std::cout << "LOG " << HipID << " -> " << std::setprecision(10)
            << "RA (Rad) : " << star.ra_rad << ", DEC (Rad) : " << star.dec_rad
            << ", Mag : " << star.mag << "\n";
  return;
}
