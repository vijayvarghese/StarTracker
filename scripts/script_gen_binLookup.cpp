#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

struct StarPair {
  std::string id1;
  std::string id2;
};

std::unordered_map<int, std::vector<StarPair>> lookup;

void write_to_bin(
    const std::string &name,
    const std::unordered_map<int, std::vector<StarPair>> &lookup) {
  std::ofstream out(name, std::ios::binary);
  if (!out)
    return;

  size_t mapSize = lookup.size();
  std::cout << "Lookup Map size : " << sizeof(size_t)
            << " , Entry Count : " << mapSize << std::endl;
  out.write(reinterpret_cast<const char *>(&mapSize), sizeof(mapSize));

  int idx = 0;
  for (const auto &[key, vec] : lookup) {
    out.write(reinterpret_cast<const char *>(&key), sizeof(key));

    size_t vecSize = vec.size();
    out.write(reinterpret_cast<const char *>(&vecSize), sizeof(vecSize));

    for (const auto &sp : vec) {
      size_t s1Len = sp.id1.size();
      out.write(reinterpret_cast<const char *>(&s1Len), sizeof(s1Len));
      out.write(sp.id1.data(), s1Len);

      size_t s2Len = sp.id2.size();
      out.write(reinterpret_cast<const char *>(&s2Len), sizeof(s2Len));
      out.write(sp.id2.data(), s2Len);
    }
    idx++;
    std::cout << "\rMaps Written to bin : " << idx;
    std::cout.flush();
  }
  std::cout << "\nDone ! -> " << idx << std::endl;
}

void loadBin(const std::string &filename,
             std::unordered_map<int, std::vector<StarPair>> &lookup) {
  std::ifstream in(filename, std::ios::binary);
  if (!in)
    return;

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
}

int main(int argc, char *argv[]) {

  if (argc < 4) {
    std::cerr << "Usage: Bin_Gen <input.csv> <output.bin> <fov_degrees>"
              << std::endl;
    return -1;
  }

  const std::string input_file = argv[1];
  const std::string output_file = argv[2];
  const double fov = std::stod(argv[3]);

  std::cout << "FOV filter: " << fov << " degrees" << std::endl;

  // --- Load CSV ---
  auto t1 = std::chrono::high_resolution_clock::now();
  std::ifstream file(input_file);
  if (!file.is_open()) {
    std::cerr << "Error: Could not open file: " << input_file << std::endl;
    return 1;
  }

  std::string line;
  if (!std::getline(file, line))
    return 0; // skip header

  int total = 0, skipped = 0;
  while (std::getline(file, line)) {
    if (line.empty())
      continue;

    std::stringstream ss(line);
    std::string cell;
    std::vector<std::string> row;
    while (std::getline(ss, cell, ','))
      row.push_back(cell);

    if (row.size() >= 3) {
      try {
        double val = std::stod(row[2]);
        total++;

        // Skip pairs whose separation exceeds FOV
        if (val > fov) {
          skipped++;
          continue;
        }

        int bin = (int)std::round(val / 0.01);
        lookup[bin].push_back({row[0], row[1]});
        std::cout << "Test Bin " << bin << "\n";

      } catch (const std::invalid_argument &e) {
        continue;
      }
    }
  }

  auto t2 = std::chrono::high_resolution_clock::now();
  std::cout << "CSV load: " << std::chrono::duration<double>(t2 - t1).count()
            << " sec" << std::endl;
  std::cout << "Total pairs: " << total << " | Skipped (> " << fov
            << "°): " << skipped << " | Kept: " << (total - skipped)
            << std::endl;

  // --- Write binary ---
  write_to_bin(output_file, lookup);
  lookup.clear();

  // --- Verify binary ---
  t1 = std::chrono::high_resolution_clock::now();
  loadBin(output_file, lookup);
  t2 = std::chrono::high_resolution_clock::now();
  std::cout << "Binary load: " << std::chrono::duration<double>(t2 - t1).count()
            << " sec" << std::endl;

  // Spot check
  auto it = lookup.find(995);
  if (it == lookup.end())
    std::cout << "Key 995 not found\n";
  else if (it->second.empty())
    std::cout << "Key 995 exists but vector is empty\n";
  else
    std::cout << "Spot check key 995: " << it->second[0].id1 << " , "
              << it->second[0].id2 << std::endl;

  return 0;
}