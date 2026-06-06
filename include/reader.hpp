// Reader hpp
#pragma once
//#include<iostream>
#include<opencv2/opencv.hpp>
#include<atomic>
#include<hal/ICamera.hpp>


void image_reader_thread(std::atomic<std::shared_ptr<cv::Mat>>& latest_frame, ST::ICamera& cam);