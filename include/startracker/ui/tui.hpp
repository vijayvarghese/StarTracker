#pragma once
#include <atomic>
#include <string>
#include <thread>

class TuiThread {
public:
  TuiThread(const std::string &name, const std::string &version,
            std::atomic<bool> &running);
  void start();
  void join();

private:
  std::string name_;
  std::string version_;
  std::thread thread_;
  std::atomic<bool> &running_;
  void run();
};