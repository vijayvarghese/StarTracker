// Reader
#include<iostream> //debug cout @ 33
#include<thread>
#include<chrono>
#include<mutex>
#include<atomic>
#include<opencv2/opencv.hpp>

#include "include/globals.h"
#include "include/reader.hpp"

static constexpr double TRACKER_HZ = 33.0;//tracker frequency in Hz
static constexpr auto TRACKER_PERIOD = std::chrono::milliseconds(static_cast<int>(1000.0 / TRACKER_HZ));
const int reader_ms = static_cast<int>(1000.0 / TRACKER_HZ);//reader sleep time if no valid frame


const std::string path_to_frame = "/tmp/startracker_frame.png";
const int expected_width = 256;
const int expected_height = 256;



void image_reader_thread(cv::Mat &frameout, std::atomic<bool> &f_readyFlag){
    
    cv::Mat img;
    while (running.load())
    {
        auto next = std::chrono::steady_clock::now() + TRACKER_PERIOD;
        std::this_thread::sleep_until(next);//sync to period

        
        img = cv::imread(path_to_frame, cv::IMREAD_COLOR);
        if (img.empty()){
            std::this_thread::sleep_for(std::chrono::milliseconds(reader_ms));
            continue;  
        }
        if ((img.cols != expected_width) || (img.rows != expected_height)){
            std::this_thread::sleep_for(std::chrono::milliseconds(reader_ms));
            continue;
        }
        
        { // Critical section.. 
        std::lock_guard<std::mutex> locklatest(M_latestframe);
        frameout = img.clone();
        //std::cout<<"frame updated (Reader) "<<"  RES : "<<img.cols<<" x "<<img.rows<<"\n"; //for debug
        f_readyFlag = true;
        }
    }
    std::cout << "[Reader] exiting.\n"; //debug exit
    {
    std::lock_guard<std::mutex> locklatest(M_latestframe);
    frameout = cv::Mat(); //invalidating frame on exit
    }
}