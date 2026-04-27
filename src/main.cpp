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


#include "reader.hpp"
#include "processor.hpp"
#include "globals.h"
#include "config.hpp"
#include "logger.hpp"



cv::Mat latestframe;
std::mutex M_latestframe;
std::atomic<bool> running{true};
std::atomic<bool> frameready{false};
std::unordered_map<int, std::vector<StarPair>> lookup;


//debug flags from runtime args
std::atomic<bool> processor_centeroid_debug = false; //debug flag for centroid and preprocessing
std::atomic<bool> processor_ray_debug = false;
std::atomic<bool> processor_angSep_debug = false;
std::atomic<bool> processor_img_debug = false;
std::atomic<bool> processor_AngProfile_debug= false;
std::atomic<int> log_level = 0;



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
        //std::cout<<"\nSIGINT received, stopping threads...\n";
        LOG_INFO<<"SIGINT received, stopping threads...";
        running.store(false);
        //LOG_FATAL<<"Test Abort";
    }
}


int main(int argc, char* argv[]){

    //implement cap_arg

    cap_args(argc, argv);

    signal(SIGINT, signal_handler);

    loadBin(lookup_cfg.binpath, lookup);
    
//debug lookup load test.

// auto it = lookup.find(134);

// if (it == lookup.end()) {
//     std::cout << "Key 134 not found\n";
// } 
// else if (it->second.empty()) {
//     std::cout << "Key 134 exists but vector is empty\n";
// } 
// else {
//     std::cout << it->second[0].id1 
//               << " , " 
//               << it->second[0].id2 
//               << std::endl;
// }
//debug lookup load test end !!!

    std::thread reader(image_reader_thread, std::ref(latestframe),std::ref(frameready));
    std::thread processor(processor_thread);
    
    if(reader.joinable()) reader.join();
    if(processor.joinable())processor.join();

    //std::cout<<"Exited !!! \n";
    LOG_INFO<<"Exited !!! ";
    return 0;
}