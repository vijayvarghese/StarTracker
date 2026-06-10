// Reader hpp
#pragma once
// #include<iostream>
#include "startracker/hal/ICamera.hpp"
#include <atomic>
#include <opencv2/opencv.hpp>

void image_reader_thread(std::atomic<std::shared_ptr<cv::Mat>> &latest_frame,
                         ST::ICamera &cam, std::atomic<bool> &running);