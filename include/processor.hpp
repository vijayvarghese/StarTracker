#pragma once
#include "startracker/core/types.hpp"
#include <atomic>
#include <opencv2/opencv.hpp>

void processor_thread(std::atomic<std::shared_ptr<cv::Mat>> &latest_frame,
                      ST::BinnedStarMap &lookup, std::atomic<bool> &running,
                      ST::StarProfileMap &rl_DAT_lookup);
