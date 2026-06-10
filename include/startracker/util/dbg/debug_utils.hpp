#pragma once
#include <atomic>
#include <opencv2/core.hpp> //Fix Me cv::types maybe enough

namespace ST::dbg {
extern std::atomic<bool> centroid;    // processor_centroid_debug
extern std::atomic<bool> ray;         // processor_ray_debug
extern std::atomic<bool> ang_sep;     // processor_angSep_debug
extern std::atomic<bool> img;         // processor_img_debug
extern std::atomic<bool> ang_profile; // processor_AngProfile_debug

/**
 * @brief Will export
 *
 * @param star_centroids
 * @param frame
 * @return cv::Mat
 */
cv::Mat
debug_pngexport_centroids(const std::vector<cv::Point2d> &star_centroids,
                          const cv::Mat &frame);

} // namespace ST::dbg
