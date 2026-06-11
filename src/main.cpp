#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#ifdef ST_TUI_BUILD
#include "startracker/ui/tui.hpp"
#endif

#include "startracker/catalog/loadbin.hpp"
#include "startracker/core/logger.hpp"
#include "startracker/core/types.hpp"
#include "startracker/hal/ICamera.hpp"
#include "startracker/hal/UnityMockFile.hpp"
#include "startracker/hal/UnityMockTCP.hpp"
#include "startracker/util/dbg/debug_utils.hpp"

#include "processor.hpp"
#include "reader.hpp"

#include "config.hpp"

namespace ST {
std::atomic<bool> running{true};
std::unordered_map<int, std::vector<StarPair>> lookup;
} // namespace ST

extern lookupconfig lookup_cfg;
extern ReaderConfig reader_cfg;

std::atomic<bool> ST::dbg::centroid = false;
std::atomic<bool> ST::dbg::ray = false;
std::atomic<bool> ST::dbg::ang_sep = false;
std::atomic<bool> ST::dbg::img = false;
std::atomic<bool> ST::dbg::ang_profile = false;
std::atomic<int> ST::core::log::log_level = 0;

std::atomic<std::shared_ptr<cv::Mat>> latest_frame{nullptr};
// auto cam = std::make_unique<ST::UnityMockTCP>();
auto cam = std::make_unique<ST::UnityMockFile>(reader_cfg.file_path);
bool use_tui = false;

// function to capture runtime args
void cap_args(int &argc, char **argv) {
  if (argc == 1)
    return;

  for (int i = 1; i < argc; i++) {
    std::string arg = argv[i];
    // std::cout<<"Arg count : "<<argc<<"  Arg : "<<arg<<std::endl;
    if (arg == "--debugc")
      ST::dbg::centroid = true;
    else if (arg == "--debugr")
      ST::dbg::ray = true;
    else if (arg == "--debuga")
      ST::dbg::ang_sep = true;
    else if (arg == "--debugimg")
      ST::dbg::img = true;
    else if (arg == "--debugap")
      ST::dbg::ang_profile = true;
    else if (arg == "--tui")
      use_tui = true;

    else if (arg == "--log0")
      ST::core::log::log_level = 0;
    else if (arg == "--log1")
      ST::core::log::log_level = 1;
    else if (arg == "--log2")
      ST::core::log::log_level = 2;
    else if (arg == "--log3")
      ST::core::log::log_level = 3;
    else if (arg == "--log4")
      ST::core::log::log_level = 4;
  }
}

// SIGINT handler function to set false on running atomic bool "running".
void signal_handler(int signal) {
  if (signal == SIGINT) {
    ST::running.store(false);
    // Camera Shutdown request !
    // cam->requestShutdown();
  }
}

int main(int argc, char *argv[]) {

  // SigINT handler
  // Lookup sigaction implimentation
  signal(SIGINT, signal_handler);

  // cap_arg
  cap_args(argc, argv);

  // Config LOad
  (void)config_init();

  // Loading lookup bin
  (void)ST::catalog::loadBin(lookup_cfg.binpath, ST::lookup);

  std::vector<std::thread> threads;
  try {
    threads.emplace_back(image_reader_thread, std::ref(latest_frame),
                         std::ref(*cam), std::ref(ST::running));
    threads.emplace_back(processor_thread, std::ref(latest_frame),
                         std::ref(ST::lookup), std::ref(ST::running));
  } catch (const std::exception &e) {
    ST::running.store(false);
    for (auto &t : threads)
      if (t.joinable())
        t.join();
    std::cerr << "Reader And Processor" << e.what() << '\n';
    throw;
  }

  try {
#ifdef ST_TUI_BUILD
    LOG_INFO << "TUI BUILD !";
    if (use_tui) {
      TuiThread tui("StarTracker", "1.0.0", ST::running);
      tui.start();
      tui.join();
      ST::running.store(false);
    } else {
      while (ST::running.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }
#else
    while (ST::running.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
#endif
  } catch (const std::exception &e) {
    ST::running.store(false);
    for (auto &t : threads)
      if (t.joinable())
        t.join();
    std::cerr << "TUI" << e.what() << '\n';
    throw;
  }

  for (auto &t : threads)
    if (t.joinable())
      t.join();
  cam->close();
  LOG_INFO << "Exited !!! ";
  return 0;
}