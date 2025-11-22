#pragma once
#include<string>
#include<chrono>


static constexpr double READER_HZ = 33.0;
static constexpr auto READER_PERIOD = std::chrono::milliseconds(static_cast<int>(1000.0 / READER_HZ));

static constexpr double TRACKER_HZ = 1.0;
static constexpr auto PROCESSOR_PERIOD = std::chrono::milliseconds(static_cast<int>(1000.0 / TRACKER_HZ));


struct ReaderConfig{
    std::string source = "file";
    std::string file_path = "/tmp/startracker_frame.png";
    int expected_width = 256;
    int expected_height = 256;
    double frequency = 33.0;
    std::chrono::milliseconds period = READER_PERIOD;
};

struct cvConfig
{
    int threshold = 200;
    int blur_ksize = 5;
    double blur_sigma = 1.0;
};


struct ProcessorConfig {
    double frequency = 1.0;
    cvConfig cv;
    std::chrono::milliseconds period = PROCESSOR_PERIOD;
};


extern const ReaderConfig reader_cfg;
extern const ProcessorConfig processor_cfg;