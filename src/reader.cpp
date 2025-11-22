// Reader
#include "include/reader.hpp"
#include<iostream> //debug cout @ 33
#include<thread>

extern cv::Mat latestframe;
extern std::mutex M_latestframe;
extern std::atomic<bool> running;
extern std::atomic<bool> frameready;


std::string path_to_frame = "/tmp/startracker_frame.png";
int expected_width = 256;
int expected_height = 256;
int reader_ms = 30;


void image_reader_thread(cv::Mat &frameout, std::atomic<bool> &f_readyFlag){
    cv::Mat img;
    while (running)
    {
        img = cv::imread(path_to_frame, cv::IMREAD_COLOR);
        if (img.empty()){
            continue;  
        }
        if ((img.cols != expected_width) || (img.rows != expected_height)){
            continue;
        }
        
        { // Critical section.. 
        std::lock_guard<std::mutex> locklatest(M_latestframe);
        frameout = img.clone();
        //std::cout<<"frame updated (Reader) "<<"  RES : "<<img.cols<<" x "<<img.rows<<"\n"; //for debug
        f_readyFlag = true;
        }


        std::this_thread::sleep_for(std::chrono::milliseconds(reader_ms));
    }
    
}