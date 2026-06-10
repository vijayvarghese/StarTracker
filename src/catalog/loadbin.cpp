#include<unordered_map>
#include<fstream>
#include<vector>
#include "startracker/core/logger.hpp"
#include "startracker/core/types.hpp"
#include "startracker/catalog/loadbin.hpp"


namespace ST::catalog
{
/**
 * @brief Loads binned lookup table (.bin) file and parse the the table, populates the global lookup veriable.
 * @param filename Reference to the .bin file path string.
 * @param lookup Reference to the global lookup veriable.(std::unordered_map<int, std::vector<StarPair>>)
 * @returns bin_load_ok or bin_load_err  
 */
BinLoadStatus loadBin(const std::string& filename,
                std::unordered_map<int, std::vector<StarPair>>& lookup)
{
    std::ifstream in(filename, std::ios::binary);
    if (!in) {
    //std::cout<<"Err LOading bin !!!!!!!"<<std::endl;   
    LOG_ERROR << "Error LOading BIN !!!"; 
        return BinLoadStatus::Error;
    }
    lookup.clear();

    size_t mapSize;
    in.read(reinterpret_cast<char*>(&mapSize), sizeof(mapSize));

    for (size_t i = 0; i < mapSize; ++i) {
        int key;
        in.read(reinterpret_cast<char *>(&key), sizeof(key));

        size_t vecSize;
        in.read(reinterpret_cast<char*>(&vecSize), sizeof(vecSize));

        std::vector<StarPair> vec(vecSize);

        for (size_t j = 0; j < vecSize; ++j) {
            size_t len1, len2;

            in.read(reinterpret_cast<char*>(&len1), sizeof(len1));
            vec[j].id1.resize(len1);
            in.read(&vec[j].id1[0], len1);

            in.read(reinterpret_cast<char*>(&len2), sizeof(len2));
            vec[j].id2.resize(len2);
            in.read(&vec[j].id2[0], len2);
        }

        lookup.emplace(key, std::move(vec));
    }
    return BinLoadStatus::Ok;
}
} //namespace ST::catalog