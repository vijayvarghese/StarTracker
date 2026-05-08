// Reader hpp
#pragma once
//#include<iostream>
#include<opencv2/opencv.hpp>
#include<atomic>


void image_reader_thread(std::atomic<std::shared_ptr<cv::Mat>>& latest_frame);