#pragma once
#include<opencv2/opencv.hpp>
#include<atomic>

void processor_thread(std::atomic<std::shared_ptr<cv::Mat>>& latest_frame);