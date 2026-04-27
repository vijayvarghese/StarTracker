#include <iostream>
#include <chrono>
#include <cstdlib>

#include "logger.hpp"
#include "globals.h"

std::mutex LogMessage::log_mutex_;
extern std::atomic<int> log_level;


static const char* level_to_string(LogLevel lvl){

    switch (lvl){
        case LogLevel::DEBUG : return "DEBUG";
        case LogLevel::ERROR : return "ERROR";
        case LogLevel::INFO : return "INFO";
        case LogLevel::WARN : return "WARN";
        case LogLevel::FATAL : return "FATAL";
        default : return "";
    }
}


// Using initializer list to create object with parameter initialized (Faster than creating object then assigning param) 
LogMessage::LogMessage(LogLevel level, const char* file, int line) : level_(level), file_(file), line_(line){}

std::ostringstream& LogMessage::stream(){
    return stream_;
}


LogMessage::~LogMessage()
{
    // 1. Severity filter

    std::lock_guard<std::mutex> lock(log_mutex_);

    auto now = std::chrono::system_clock::now();
    auto ms  = std::chrono::duration_cast<std::chrono::milliseconds>(
                   now.time_since_epoch()).count();

    switch(log_level.load())
    {
        case 0:
            std::cerr << stream_.str() << "\n";
            break;

        case 1:
            std::cerr << "["<<level_to_string(level_)<<"] "
                      << stream_.str() << "\n";
            break;

        case 2:
            std::cerr << "["<<ms<<"] "
                      << "["<<level_to_string(level_)<<"] "
                      << stream_.str() << "\n";
            break;

        case 3:
            std::cerr << "["<<level_to_string(level_)<<"] "
                      << "["<<file_<<":"<<line_<<"] "
                      << stream_.str() << "\n";
            break;

        case 4:
            std::cerr << "["<<ms<<"] "
                      << "["<<level_to_string(level_)<<"] "
                      << "["<<file_<<":"<<line_<<"] "
                      << stream_.str() << "\n";
            break;
        default:
            std::cerr << stream_.str() << "\n";
            break;
    }

    if(level_ == LogLevel::FATAL)
        std::abort();
}


