#pragma once
#include <string>
#include <thread>

class TuiThread {
public:
    TuiThread(const std::string& name, const std::string& version);
    void start();
    void join();

private:
    std::string name_;
    std::string version_;
    std::thread thread_;
    void run();
};