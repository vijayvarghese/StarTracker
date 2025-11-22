// Processing Thread
#include<iostream>
#include<opencv2/opencv.hpp>
#include<thread>
#include<chrono>
#include<atomic>
#include<mutex>

#include "include/globals.h"
#include "include/processor.hpp"


//int processor_ms = 1000;
static constexpr double TRACKER_HZ = 1.0;
static constexpr auto TRACKER_PERIOD = std::chrono::milliseconds(static_cast<int>(1000.0 / TRACKER_HZ));



cv::Mat get_frame_safe(){ //critial section (return the clone of the latestframe as cv::Mat) implimentation @33
    std::lock_guard<std::mutex> local_cpy_lock(M_latestframe);
    if(latestframe.empty()) return cv::Mat();
    return latestframe.clone();
}

void preprocess_frame(cv::Mat &frame, double &ts){ // preporcessing each frame
    cv::Mat gray, blurred, bw;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY); //gray scale
    cv::GaussianBlur(gray, blurred, cv::Size(5,5), 1.0); //induced blur
    cv::threshold(blurred, bw, 200, 255, cv::THRESH_BINARY); //any dot above 200 gets full brightness 255 rest full black.
    int bright = cv::countNonZero(bw); // count white dots

    std::cout << "[Tracker] t=" << ts << "s | size="
              << frame.cols << "x" << frame.rows
              << " | bright_pixels=" << bright << std::endl;
}

void processor_thread(){ 
    //  !frameready.load()continue -> get_frame_safe() -> frame_cpy_local.empty()continue -> preprocess_frame()
    double tsec = 0.0;
    while (running.load())
    {
        auto next = std::chrono::steady_clock::now() + TRACKER_PERIOD;
        std::this_thread::sleep_until(next);

        if(!frameready.load()){
        std::cout << "[Tracker] No frame ready yet, skipping tick.\n";
        next += TRACKER_PERIOD;
        tsec += std::chrono::duration_cast<std::chrono::duration<double>>(TRACKER_PERIOD).count();
        continue;
        }

        cv::Mat frame_cpy_local;
        { //Critical Section safe function call @16
            frame_cpy_local = get_frame_safe();
        }

        if (frame_cpy_local.empty()) {
            std::cout << "[Tracker] Frame was empty after grabbing, skipping.\n";
            next += TRACKER_PERIOD;
            tsec += std::chrono::duration_cast<std::chrono::duration<double>>(TRACKER_PERIOD).count();
            continue;
        }

        preprocess_frame(frame_cpy_local, tsec); //processing level -0

        //advance time with period for next wait_until(next)
        next += TRACKER_PERIOD;
        tsec += std::chrono::duration_cast<std::chrono::duration<double>>(TRACKER_PERIOD).count();
        //std::cout<<"Processor Frame : "<<frame_cpy_local.cols<<" x "<<frame_cpy_local.rows<<"\n";
    }
    std::cout << "[Tracker] exiting.\n"; //debug exit
}