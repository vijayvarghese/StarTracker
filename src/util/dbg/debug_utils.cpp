#include "startracker/util/dbg/debug_utils.hpp"
#include "startracker/core/logger.hpp"
#include "startracker/core/types.hpp"
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

void printHypotheses(const std::vector<StarHypothesis> &hypotheses) {
  LOG_DEBUG << "Hypotheses (" << hypotheses.size() << "):\n";

  for (size_t i = 0; i < hypotheses.size(); ++i) {
    const auto &h = hypotheses[i];

    LOG_DEBUG << "[" << i << "]\n"
              << "  centroid          : (" << h.centroid.x << ", "
              << h.centroid.y << ")\n"
              << "  ray               : (" << h.ray[0] << ", " << h.ray[1]
              << ", " << h.ray[2] << ")\n"
              << "  hip_id            : " << h.hip_id << "\n"
              << "  vote_count        : " << h.vote_count << "\n"
              << "  second_vote_count : " << h.second_vote_count << "\n"
              << "  second_best_id    : " << h.hip_id_second_best << "\n"
              << "  confident         : " << std::boolalpha << h.confident
              << "\n\n";
  }
}

} // namespace ST::dbg