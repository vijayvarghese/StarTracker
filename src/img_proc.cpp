#include <opencv2/imgproc.hpp>
#include "img_proc.hpp"
#include "config.hpp"
#include "globals.h"
#include "logger.hpp"



cv::Mat preprocess_frame(cv::Mat &frame, const double &ts){
    cv::Mat gray, blurred, bw;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY); //gray scale
    cv::GaussianBlur(gray, blurred, cv::Size(processor_cfg.cv.blur_ksize,processor_cfg.cv.blur_ksize), processor_cfg.cv.blur_sigma); //induced blur
    cv::threshold(blurred, bw, processor_cfg.cv.threshold, 255, cv::THRESH_BINARY); //any dot above 200 gets full brightness 255 rest full black.
    int bright = cv::countNonZero(bw); // count white dots
    if (ST::dbg::centroid){
    LOG_DEBUG << "[Tracker] [Preprocessing] t=" << ts << "s | size="
              << frame.cols << "x" << frame.rows
              << " | bright_pixels=" << bright;
    }
    return bw;
}

std::vector<cv::Point2d> get_centroids(const cv::Mat &BW, const double &ts){
    //gets centroids into a 2d vector and returns the same... {area_threshold, }
    cv::Mat labels, stats, centroids;
    int n = cv::connectedComponentsWithStats(BW, labels, stats, centroids);

    std::vector<cv::Point2d> temp_centroid;

    for (int i = 1; i<n; i++){
        int area = stats.at<int>(i, cv::CC_STAT_AREA);
        if (area < 1 || area > processor_cfg.cv.area_max)continue;
        
        double cx = centroids.at<double>(i,0);
        double cy = centroids.at<double>(i,1);
        temp_centroid.emplace_back(cx,cy);
    }
    if(ST::dbg::centroid){
    LOG_DEBUG << "[Tracker] [Centroid] t=" << ts
              << " | stars=" << temp_centroid.size()
              << " | size=" << BW.cols << "x" << BW.rows;
    }

    // just to print the centroids on console....
    if (ST::dbg::centroid){
    int idx = 0;
    for (const auto& c : temp_centroid) {
    LOG_DEBUG << "[Tracker] [Centroid]"<< "Centroid[" << idx++ << "] = ("
              << c.x << ", " << c.y << ")";
    }
    }
    //the above line of code from int idx is debug....
    
              

    return temp_centroid;
}

