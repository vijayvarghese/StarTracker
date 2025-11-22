#pragma once
#include<opencv2/opencv.hpp>
#include<atomic>
#include<mutex>

extern cv::Mat latestframe;
extern std::mutex M_latestframe;
extern std::atomic<bool> running;
extern std::atomic<bool> frameready;
