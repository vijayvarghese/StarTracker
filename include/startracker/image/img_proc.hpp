#pragma once
#include <opencv2/core/core.hpp>

namespace ST::image
{
    
/**
 * @brief Preprocess frame. process blur, grayscaling, and thresholding
 * @param frame Reference to the cv::Mat frame to be processed.
 * @param ts Reference to the tic counter (debug timestamping).
 * @return cv::Mat preprocessed frame.  
 */
cv::Mat preprocess_frame(cv::Mat &frame, const double &ts, 
    const int& blur_ksize, 
    const double& blur_sigma,
    const int& threshold 
    );


/**
 * @brief Process and find centroids with cv::connectedComponentsWithStats, max area filtering with MaxArea
 * global config.
 * @param BW Preprocessed cv::Mat frame reference, from which centroids are found;
 * @param ts Reference to the tic counter (debug timestamping)
 * @return std::vector<cv::Point2d> - vector of centroids. 
 */
std::vector<cv::Point2d> get_centroids(const cv::Mat &BW, const double &ts, const int& area_max);

} //namespace ST::image