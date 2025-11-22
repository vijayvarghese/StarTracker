// Reader hpp
#pragma once
//#include<iostream>
#include<opencv2/opencv.hpp>
#include<atomic>


void image_reader_thread(cv::Mat &frameout, std::atomic<bool> &f_readyFlag);