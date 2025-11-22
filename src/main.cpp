#include<iostream>
#include<opencv2/opencv.hpp>
#include<atomic>
#include<thread>

#include "include/reader.hpp"
#include "include/processor.hpp"


cv::Mat latestframe;
std::mutex M_latestframe;
std::atomic<bool> running{true};
std::atomic<bool> frameready{false};


int main(){


    std::thread reader(image_reader_thread, std::ref(latestframe),std::ref(frameready));
    std::thread processor(processor_thread);
    
    if(reader.joinable()) reader.join();
    if(processor.joinable())processor.join();

    std::cout<<"Exited !!! \n";
    return 0;
}