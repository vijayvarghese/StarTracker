#include<iostream>
#include<opencv2/opencv.hpp>
#include<atomic>
#include<thread>
#include<mutex>
#include<chrono>
#include<csignal>
#include<fstream>
#include<vector>
#include<string>
#include<unordered_map>
#include<fstream>
#include<memory>

#include"nlohmann/json.hpp"
#include"tui.hpp"


#include "reader.hpp"
#include "processor.hpp"
#include "globals.h"
#include "config.hpp"
#include "logger.hpp"



//cv::Mat latestframe;
//std::mutex M_latestframe;
//std::shared_ptr<cv::Mat> latest_frame;
std::atomic<bool> running{true};
//std::atomic<bool> frameready{false};
std::unordered_map<int, std::vector<StarPair>> lookup;
std::atomic<std::shared_ptr<cv::Mat>> latest_frame{nullptr};



//debug flags from runtime args
std::atomic<bool> processor_centeroid_debug = false; //debug flag for centroid and preprocessing
std::atomic<bool> processor_ray_debug = false;
std::atomic<bool> processor_angSep_debug = false;
std::atomic<bool> processor_img_debug = false;
std::atomic<bool> processor_AngProfile_debug= false;
std::atomic<int> log_level = 0;
bool use_tui = false;



// Function definition for loading star catalog with starID and angle of separation to lookup Umap. 
void loadBin(const std::string& filename,
                std::unordered_map<int, std::vector<StarPair>>& lookup)
{
    std::ifstream in(filename, std::ios::binary);
    if (!in) {
    //std::cout<<"Err LOading bin !!!!!!!"<<std::endl;   
    LOG_ERROR << "Error LOading BIN !!!"; 
        return;
    }
    lookup.clear();

    size_t mapSize;
    in.read(reinterpret_cast<char*>(&mapSize), sizeof(mapSize));

    for (size_t i = 0; i < mapSize; ++i) {
        int key;
        in.read(reinterpret_cast<char*>(&key), sizeof(key));

        size_t vecSize;
        in.read(reinterpret_cast<char*>(&vecSize), sizeof(vecSize));

        std::vector<StarPair> vec(vecSize);

        for (size_t j = 0; j < vecSize; ++j) {
            size_t len1, len2;

            in.read(reinterpret_cast<char*>(&len1), sizeof(len1));
            vec[j].id1.resize(len1);
            in.read(&vec[j].id1[0], len1);

            in.read(reinterpret_cast<char*>(&len2), sizeof(len2));
            vec[j].id2.resize(len2);
            in.read(&vec[j].id2[0], len2);
        }

        lookup.emplace(key, std::move(vec));
    }
}

//function to capture runtime args
void cap_args(int& argc, char** argv){
    if (argc==1) return;

    for (int i = 1; i<argc; i++){
        std::string arg = argv[i];
        //std::cout<<"Arg count : "<<argc<<"  Arg : "<<arg<<std::endl;
        if (arg == "--debugc") processor_centeroid_debug = true;
        else if (arg == "--debugr") processor_ray_debug = true; 
        else if (arg == "--debuga") processor_angSep_debug = true;
        else if (arg == "--debugimg") processor_img_debug = true;
        else if (arg == "--debugap") processor_AngProfile_debug = true;
        else if (arg == "--tui") use_tui = true;

        else if (arg == "--log0") log_level = 0;
        else if (arg == "--log1") log_level = 1;
        else if (arg == "--log2") log_level = 2;
        else if (arg == "--log3") log_level = 3;
        else if (arg == "--log4") log_level = 4;
    }
}



//SIGINT handler function to set false on running atomic bool "running".
void signal_handler(int signal){
    if(signal == SIGINT){
        running.store(false);
    }
}

nlohmann::json load_config_json(){
    std::ifstream f("../../config/config.json");
    return nlohmann::json::parse(f);;
}


int main(int argc, char* argv[]){

    //SigINT handler 
    //Lookup sigaction implimentation
    signal(SIGINT, signal_handler);

    //cap_arg
    cap_args(argc, argv);

    //Config LOad
    try {
        auto config = load_config_json();
        //std::cout << "Config load test -  " << config["reader"]["file"]["path"] << "\n";
    }
    catch (...) {
        std::cerr << "An unknown error occurred while loading and parsing the json config file ! " << std::endl;
    }

    //std::atomic<std::shared_ptr<cv::Mat>> latest_frame;

    //Loading lookup bin
    loadBin(lookup_cfg.binpath, lookup);
    
    //
    std::vector<std::thread> threads;
    try
    {
        threads.emplace_back(image_reader_thread, std::ref(latest_frame));
        threads.emplace_back(processor_thread, std::ref(latest_frame));
    }
    catch(const std::exception& e)
    {
        running.store(false);
        for (auto& t : threads)
            if (t.joinable()) t.join();
        std::cerr << "Reader And Processor" << e.what() << '\n';
        throw;
    }
    

    try {
        if (use_tui) {
            TuiThread tui("StarTracker", "1.0.0");
            tui.start();
            tui.join();
            running.store(false);
        } else {
            while (running.load()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
    }
    catch (const std::exception& e) {
        running.store(false);
        for (auto& t : threads)
            if (t.joinable()) t.join();
        std::cerr << "TUI" << e.what() << '\n';
        throw;
    }


    for (auto& t : threads)
        if (t.joinable()) t.join();


    LOG_INFO<<"Exited !!! ";
    return 0;
}