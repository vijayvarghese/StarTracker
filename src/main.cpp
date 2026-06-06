#include<iostream>
#include<atomic>
#include<thread>
#include<chrono>
#include<csignal>
#include<fstream>
#include<vector>
#include<string>
#include<unordered_map>
#include<memory>


#include"tui.hpp"


#include "reader.hpp"
#include "processor.hpp"
#include "globals.h"
#include "config.hpp"
#include "logger.hpp"
#include "types.hpp"
#include "core.hpp"
#include "hal/ICamera.hpp"
#include "hal/UnityMock.hpp"


std::atomic<bool> ST::running{true};
std::unordered_map<int, std::vector<StarPair>> ST::lookup;



std::atomic<bool> ST::dbg::centroid = false;
std::atomic<bool> ST::dbg::ray = false;
std::atomic<bool> ST::dbg::ang_sep = false;
std::atomic<bool> ST::dbg::img = false;
std::atomic<bool> ST::dbg::ang_profile = false;
std::atomic<int> ST::log::log_level = 0;

std::atomic<std::shared_ptr<cv::Mat>> latest_frame{nullptr}; 
bool use_tui = false;


//function to capture runtime args
void cap_args(int& argc, char** argv){
    if (argc==1) return;

    for (int i = 1; i<argc; i++){
        std::string arg = argv[i];
        //std::cout<<"Arg count : "<<argc<<"  Arg : "<<arg<<std::endl;
        if (arg == "--debugc") ST::dbg::centroid = true;
        else if (arg == "--debugr") ST::dbg::ray = true; 
        else if (arg == "--debuga") ST::dbg::ang_sep = true;
        else if (arg == "--debugimg") ST::dbg::img = true;
        else if (arg == "--debugap") ST::dbg::ang_profile = true;
        else if (arg == "--tui") use_tui = true;

        else if (arg == "--log0") ST::log::log_level = 0;
        else if (arg == "--log1") ST::log::log_level = 1;
        else if (arg == "--log2") ST::log::log_level = 2;
        else if (arg == "--log3") ST::log::log_level = 3;
        else if (arg == "--log4") ST::log::log_level = 4;
    }
}


//SIGINT handler function to set false on running atomic bool "running".
void signal_handler(int signal){
    if(signal == SIGINT){
        ST::running.store(false);
    }
}


/**
 * @brief Loads binned lookup table (.bin) file and parse the the table, populates the global lookup veriable.
 * @param filename Reference to the .bin file path string.
 * @param lookup Reference to the global lookup veriable.(std::unordered_map<int, std::vector<StarPair>>)
 * @returns bin_load_ok or bin_load_err  
 */
BinLoadStatus loadBin(const std::string& filename,
                std::unordered_map<int, std::vector<StarPair>>& lookup)
{
    std::ifstream in(filename, std::ios::binary);
    if (!in) {
    //std::cout<<"Err LOading bin !!!!!!!"<<std::endl;   
    LOG_ERROR << "Error LOading BIN !!!"; 
        return BinLoadStatus::Error;
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
    return BinLoadStatus::Ok;
}


int main(int argc, char* argv[]){

    //SigINT handler 
    //Lookup sigaction implimentation
    signal(SIGINT, signal_handler);

    //cap_arg
    cap_args(argc, argv);

    //Config LOad
    (void)config_init();

    //std::atomic<std::shared_ptr<cv::Mat>> latest_frame;

    //Loading lookup bin
    (void)loadBin(lookup_cfg.binpath, ST::lookup);
    
    auto cam = std::make_unique<ST::UnityMock>(reader_cfg.file_path);

    //
    std::vector<std::thread> threads;
    try
    {
        threads.emplace_back(image_reader_thread, std::ref(latest_frame), std::ref(*cam));
        threads.emplace_back(processor_thread, std::ref(latest_frame));
    }
    catch(const std::exception& e)
    {
        ST::running.store(false);
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
            ST::running.store(false);
        } else {
            while (ST::running.load()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
    }
    catch (const std::exception& e) {
        ST::running.store(false);
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