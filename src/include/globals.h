#pragma once
#include<opencv2/opencv.hpp>
#include<atomic>
#include<mutex>

struct StarPair {
    std::string id1;
    std::string id2;
};


extern std::unordered_map<int, std::vector<StarPair>> lookup;

extern cv::Mat latestframe;
extern std::mutex M_latestframe;
extern std::atomic<bool> running;
extern std::atomic<bool> frameready;
extern std::atomic<bool> processor_centeroid_debug; 
extern std::atomic<bool> processor_ray_debug;
extern std::atomic<bool> processor_angSep_debug;
extern std::atomic<bool> processor_img_debug;
extern std::atomic<int> verbose_level;