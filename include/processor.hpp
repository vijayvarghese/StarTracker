#pragma once
#include<opencv2/opencv.hpp>
#include<atomic>
#include "startracker/core/types.hpp"

void processor_thread(
    std::atomic<std::shared_ptr<cv::Mat>> &latest_frame, 
    std::unordered_map<int, std::vector<StarPair>> &lookup, 
    std::atomic<bool> &running
);
