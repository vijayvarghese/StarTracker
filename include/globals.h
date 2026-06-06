#pragma once
#include <atomic>
#include <mutex>
#include <memory>
#include "types.hpp"
#include <unordered_map>
#include <vector>




namespace ST {
    extern std::atomic<bool> running;
    extern std::unordered_map<int, std::vector<StarPair>> lookup;

    namespace dbg {
        extern std::atomic<bool> centroid; //processor_centroid_debug
        extern std::atomic<bool> ray;  //processor_ray_debug
        extern std::atomic<bool> ang_sep; //processor_angSep_debug
        extern std::atomic<bool> img; //processor_img_debug
        extern std::atomic<bool> ang_profile; //processor_AngProfile_debug
    }
    namespace log {
        extern std::atomic<int> log_level;
    }
}