// Reader
#include<iostream> //debug cout
#include<thread>
#include<chrono>
#include<mutex>
#include<atomic>
#include<opencv2/opencv.hpp>

#include "globals.h"
#include "reader.hpp"
#include "config.hpp"
#include "logger.hpp"

void image_reader_thread(cv::Mat &frameout, std::atomic<bool> &f_readyFlag){
    
    cv::Mat img;
    while (running.load())
    {
        auto next = std::chrono::steady_clock::now() + reader_cfg.period;
        std::this_thread::sleep_until(next);//sync to period

        img = cv::imread(reader_cfg.file_path, cv::IMREAD_COLOR);
        if (img.empty()){
            std::this_thread::sleep_for(std::chrono::milliseconds(reader_cfg.period));
            continue;  
        }
        if ((img.cols != reader_cfg.expected_width) || (img.rows != reader_cfg.expected_height)){
            std::this_thread::sleep_for(std::chrono::milliseconds(reader_cfg.period));
            continue;
        }
        
        { // Critical section.. 
        std::lock_guard<std::mutex> locklatest(M_latestframe);
        frameout = img.clone();
        //LOG_DEBUG<<"frame updated (Reader) "<<"  RES : "<<img.cols<<" x "<<img.rows; //for debug
        f_readyFlag = true;
        }
    }
    LOG_INFO << "[Reader] exiting.."; //debug exit
    {
    std::lock_guard<std::mutex> locklatest(M_latestframe);
    frameout = cv::Mat(); //invalidating frame on exit
    }
}