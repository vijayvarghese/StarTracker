#pragma once
#include<string>
#include<chrono>


static constexpr double READER_HZ = 33.0;
static constexpr auto READER_PERIOD = std::chrono::milliseconds(static_cast<int>(1000.0 / READER_HZ));

static constexpr double TRACKER_HZ = 1.0;
static constexpr auto PROCESSOR_PERIOD = std::chrono::milliseconds(static_cast<int>(1000.0 / TRACKER_HZ));


struct ReaderConfig{
    std::string source = "file";
    //std::string file_path = "/tmp/startracker_frame.png";
    //std::string file_path = "../docs/star_latest_Phase0Validation_Debug_.png";
    //std::string file_path = "/tmp/star_latest.png";
    std::string file_path = "../docs/Unity Frame_screenshot_04.01.2026.png";
    int expected_width = 1024;
    int expected_height = 1024;
    double frequency = 33.0;
    std::chrono::milliseconds period = READER_PERIOD;
};


struct CameraIntrinsic
{
    
   double fx = 2522;
   double fy = 2522;
   double cx = 512;
   double cy = 512;

};


struct cvConfig
{
    int threshold = 200;
    int blur_ksize = 5;
    double blur_sigma = 1.0;
    int area_max = 2000;
    bool window_debug = true;
};

struct pinhole_cam_intresic
{
    /* data */
};


struct ProcessorConfig {
    double frequency = 1.0;
    cvConfig cv;
    std::chrono::milliseconds period = PROCESSOR_PERIOD;
};

extern const CameraIntrinsic camera_intr_cfg;
extern const ReaderConfig reader_cfg;
extern const ProcessorConfig processor_cfg;