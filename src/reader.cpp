// Reader
#include<thread>
#include<chrono>
#include<atomic>
#include<opencv2/opencv.hpp>
#include<memory>


#include "reader.hpp"
#include "config.hpp"
#include "startracker/core/logger.hpp"
#include "startracker/core/types.hpp"
#include "startracker/hal/ICamera.hpp"




void image_reader_thread(
    std::atomic<std::shared_ptr<cv::Mat>>& latest_frame, 
    ST::ICamera& cam, 
    std::atomic<bool>& running
)
{
    
    cv::Mat img;
    while (running.load())
    {
        auto next = std::chrono::steady_clock::now() + reader_cfg.period;
        std::this_thread::sleep_until(next);//sync to period

        //img = cv::imread(reader_cfg.file_path, cv::IMREAD_COLOR);
        img = cam.grabFrame();
        if (img.empty()){
            std::this_thread::sleep_for(std::chrono::milliseconds(reader_cfg.period));
            continue;  
        }
        if ((img.cols != reader_cfg.expected_width) || (img.rows != reader_cfg.expected_height)){
            std::this_thread::sleep_for(std::chrono::milliseconds(reader_cfg.period));
            continue;
        }
        
        //{ // Critical section.. 
        //std::lock_guard<std::mutex> locklatest(M_latestframe);
        cv::Mat frame_in = img.clone();
        ////LOG_DEBUG<<"frame updated (Reader) "<<"  RES : "<<img.cols<<" x "<<img.rows; //for debug
        //f_readyFlag = true;
        //}

        auto new_frame = std::make_shared<cv::Mat>(std::move(frame_in));

        // publish atomically
        latest_frame.store(new_frame, std::memory_order_release);



    }
    LOG_INFO << "[Reader] exiting.."; //debug exit
    {
    //std::lock_guard<std::mutex> locklatest(M_latestframe);
    //frame_in = cv::Mat(); //invalidating frame on exit
    }
}