// Processing Thread
#include<iostream>
#include<opencv2/opencv.hpp>
#include<thread>
#include<chrono>
#include<atomic>
#include<mutex>

#include "include/globals.h"
#include "include/processor.hpp"
#include "include/config.hpp"



cv::Mat get_frame_safe(){ //critial section (return the clone of the latestframe as cv::Mat) implimentation @33
    std::lock_guard<std::mutex> local_cpy_lock(M_latestframe);
    if(latestframe.empty()) return cv::Mat();
    return latestframe.clone();
}

cv::Mat preprocess_frame(cv::Mat &frame, double &ts){ // preporcessing each frame
    cv::Mat gray, blurred, bw;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY); //gray scale
    cv::GaussianBlur(gray, blurred, cv::Size(processor_cfg.cv.blur_ksize,processor_cfg.cv.blur_ksize), processor_cfg.cv.blur_sigma); //induced blur
    cv::threshold(blurred, bw, processor_cfg.cv.threshold, 255, cv::THRESH_BINARY); //any dot above 200 gets full brightness 255 rest full black.
    int bright = cv::countNonZero(bw); // count white dots
    std::cout << "[Tracker] [Preprocessing] t=" << ts << "s | size="
              << frame.cols << "x" << frame.rows
              << " | bright_pixels=" << bright << std::endl;
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
    std::cout << "[Tracker] [Centroid] t=" << ts
              << " | stars=" << temp_centroid.size()
              << " | size=" << BW.cols << "x" << BW.rows
              << std::endl;

    // just to print the cnetroids on console....

    int idx = 0;
    for (const auto& c : temp_centroid) {
    std::cout << "[Tracker] [Centroid] centroid[" << idx++ << "] = ("
              << c.x << ", " << c.y << ")\n";
    }
    //the above line of code from int idx is debug....
    
              

    return temp_centroid;
}

cv::Mat debug_pngexport(const std::vector<cv::Point2d> &star_centroids, const cv::Mat &frame){
    int idx = 1;
    cv::Mat temp = frame.clone();
    for (auto &c : star_centroids) {
        cv::circle(temp, c, 4, cv::Scalar(0, 0, 255), 1);
        cv::putText(temp,
                std::to_string(idx),
                c,
                cv::FONT_HERSHEY_SIMPLEX,
                0.5,                    // font scale
                cv::Scalar(0, 255, 0), // text color (green for visibility)
                1                      // thickness
    );
    idx++;
    }
    // Save debug image
    //cv::imwrite("/tmp/tracker_debug.png", frame);
    return temp;
}

// Assumption - using pin hole camera model
// Assumption - body frame is alliged with the camera frame... 
//              ->  body frame z axix is same as the camera z axis (focal axis, perpenticulr to the image plane.) 

// Phase 0 Validation 
//   -> Steady frame with 2 stars
//   -> docs/star_latest_Phase0Validation_Debug_.png
//   -> Ray forming and normalizing....


cv::Vec3d pixel_to_body_ray(
    double u, double v,
    double fx, double fy,
    double cx, double cy)
{
    cv::Vec3d r;
    r[0] = (u - cx) / fx;
    r[1] = (v - cy) / fy;
    r[2] = 1.0;
    
    return cv::normalize(r);
}

// Phase 1 validation 
//   -> Angular separation validation
//   ->


// THIS IS A SAMPLE IMPLEMENTATION OF ANGLE BETWEEN CALCULATION 
// only use this with 2 centroid debug image !!!!!!!!!!!!
double angle_between(const cv::Vec3d& a, const cv::Vec3d& b)
{
    double dot = a.dot(b);
    dot = std::clamp(dot, -1.0, 1.0);
    return std::acos(dot) * 180.0 / CV_PI;
}








void processor_thread(){ 
    //  !frameready.load()continue -> get_frame_safe() -> frame_cpy_local.empty()continue -> preprocess_frame()
    double tsec = 0.0;
    auto next = std::chrono::steady_clock::now() + processor_cfg.period;

    //debug 
    bool flag_debug_png_export = false;  // to export only once 

    cv::Mat debug; 

    while (running.load())
    {
        cv::Mat frame_cpy_local, preprocess_ed_frame;
        std::vector<cv::Point2d> centroids;
        std::vector<cv::Vec3d> ray;
        std::vector<double> theta_vec;
        cv::Vec3d temp_ray; 
        std::this_thread::sleep_until(next);

        if(!frameready.load()){
            std::cout << "[Tracker] No frame ready yet, skipping tick.\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(2));//avoid busy-waiting need to change it to condition variable
            goto advance;
        }

        
        { //Critical Section safe function call @16
            frame_cpy_local = get_frame_safe();
        }

        if (frame_cpy_local.empty()) {
            std::cout << "[Tracker] Frame was empty after grabbing, skipping.\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(2));////avoid busy-waiting need to change it to condition variable
            goto advance;
        }


        preprocess_ed_frame = preprocess_frame(frame_cpy_local, tsec); //processing level -0
        centroids = get_centroids(preprocess_ed_frame, tsec);
        
        

        //optimize the below for loops 
        //  --> loop one - find camera/body ray for each centroid


        // loop one - camera/body ray for each centroid
        // returns a std::vector<cv::Vec3d> ray
        for (const auto& c : centroids) {
            std::cout << "[Tracker] [Centroid] "<< "("<< c.x << ", " << c.y << ")\n";
            temp_ray = pixel_to_body_ray(c.x,c.y,camera_intr_cfg.fx,camera_intr_cfg.fy,camera_intr_cfg.cx,camera_intr_cfg.cy);

            std::cout << "[Tracker] [Centroid] [Ray] = ("<< temp_ray[0] << ", " << temp_ray[1] << ", " <<temp_ray[2]<< ")\n";
            ray.emplace_back(temp_ray);
        }


        
        //SAMPLE Angular separation implimentation USE ONLY WITH 2 CENTROIDS ALONE !!!!!!! 
        
        //std::cout << "[Tracker] [Centroid] [Ray] = ("<< ray[0] << "), (" << ray[1] << "), Angular separation -> " <<angle_between(ray[0],ray[1])<< "\n";
        
        for (int i = 0; i <= ray.size(); i++){
            for (int j = i+1; j < ray.size(); j++){
                theta_vec.emplace_back(angle_between(ray[i],ray[j+1]));   
                std::cout << "[Tracker] [Centroid] [Ray] ["<<i<<","<<j<<"]= ("<< ray[i] << "), (" << ray[j] << "), Angular separation -> " <<angle_between(ray[i],ray[j])<< "\n";
            }
        }





        if (processor_cfg.cv.window_debug){
            cv::namedWindow("Unity Frame", cv::WINDOW_AUTOSIZE);
            cv::namedWindow("Preprocessed", cv::WINDOW_AUTOSIZE);
            cv::namedWindow("Centroid Debug", cv::WINDOW_AUTOSIZE);
            debug = debug_pngexport(centroids,frame_cpy_local);
            cv::imshow("Unity Frame", frame_cpy_local);
            cv::imshow("Preprocessed", preprocess_ed_frame);        // or gray if you prefer
            cv::imshow("Centroid Debug", debug);
            // Required for window update
            cv::waitKey(1);   // 1 ms to allow GUI to refresh
        }


        advance:
        //advance time with period for next wait_until(next)
        next += processor_cfg.period;
        tsec += std::chrono::duration_cast<std::chrono::duration<double>>(processor_cfg.period).count();
        //std::cout<<"Processor Frame : "<<frame_cpy_local.cols<<" x "<<frame_cpy_local.rows<<"\n";
    }
    std::cout << "[Tracker] exiting.\n"; //debug exit
}