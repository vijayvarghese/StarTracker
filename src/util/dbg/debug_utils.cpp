#include "startracker/util/dbg/debug_utils.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace ST::dbg {

cv::Mat
debug_pngexport_centroids(const std::vector<cv::Point2d> &star_centroids,
                          const cv::Mat &frame) {
  int idx = 1;
  cv::Mat temp = frame.clone();
  for (auto &c : star_centroids) {
    cv::circle(temp, c, 4, cv::Scalar(0, 0, 255), 1);
    cv::putText(temp, std::to_string(idx), c, cv::FONT_HERSHEY_SIMPLEX,
                0.5,                   // font scale
                cv::Scalar(0, 255, 0), // text color (green for visibility)
                1                      // thickness
    );
    idx++;
  }
  // Save debug image
  cv::imwrite("/tmp/tracker_debug.png", frame);
  return temp;
}

} // namespace ST::dbg