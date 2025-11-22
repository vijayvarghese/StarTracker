#include<iostream>
#include<opencv2/opencv.hpp>
#include<atomic>
#include<thread>
#include<mutex>
#include<chrono>
#include<csignal>

#include "include/reader.hpp"
#include "include/processor.hpp"
#include "include/globals.h"



cv::Mat latestframe;
std::mutex M_latestframe;
std::atomic<bool> running{true};
std::atomic<bool> frameready{false};

void signal_handler(int signal){
    if(signal == SIGINT){
        std::cout<<"\nSIGINT received, stopping threads...\n";
        running.store(false);
    }
}


int main(){
    signal(SIGINT, signal_handler);

    std::thread reader(image_reader_thread, std::ref(latestframe),std::ref(frameready));
    std::thread processor(processor_thread);
    
    if(reader.joinable()) reader.join();
    if(processor.joinable())processor.join();

    std::cout<<"Exited !!! \n";
    return 0;
}