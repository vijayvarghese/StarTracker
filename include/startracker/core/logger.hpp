#pragma once
#include <sstream>
#include <mutex>
#include <atomic>

namespace ST::core::log {
    extern std::atomic<int> log_level;
}

enum class LogLevel{
    DEBUG,
    INFO,
    WARN,
    ERROR,
    FATAL
};

class LogMessage {
public:
    LogMessage(LogLevel level, const char* file, int line);
    ~LogMessage();

    std::ostringstream& stream();


private:
    LogLevel level_;
    const char* file_;
    int line_;
    std::ostringstream stream_;

    static std::mutex log_mutex_;

};





#define LOG(level) LogMessage(LogLevel::level,__FILE__,__LINE__).stream()


#define LOG_DEBUG LOG(DEBUG)
#define LOG_INFO LOG(INFO)
#define LOG_WARN LOG(WARN)
#define LOG_ERROR LOG(ERROR)
#define LOG_FATAL LOG(FATAL)