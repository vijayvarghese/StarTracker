// Processing Thread
#include <opencv2/opencv.hpp>
#include <thread>
#include <chrono>
#include <atomic>
#include <unordered_map>
#include <unordered_set>



#include "processor.hpp"
#include "config.hpp"
#include "startracker/core/logger.hpp"
#include "startracker/core/types.hpp"
#include "startracker/util/dbg/debug_utils.hpp"

#include "startracker/image/img_proc.hpp"
#include "startracker/math/geometry.hpp"
#include "startracker/identification/n-star/n-star.hpp"


//Macro to print AngularSepProfile Struct.
#define PRINT_ANGULAR_SEP_PROFILE(profile_vec) \
do { \
    LOG_DEBUG << "=== AngularSeparationProfile [" << (profile_vec).size() << " profiles] ==="; \
    for (size_t _pi = 0; _pi < (profile_vec).size(); _pi++) { \
        const auto& _p = (profile_vec)[_pi]; \
        LOG_DEBUG << "  ┌─ Profile [" << _pi << "]"; \
        LOG_DEBUG << "  │  angle       : " << std::fixed << std::setprecision(4) \
                  << _p.centroid_ray_profile.angular_separation << " deg"; \
        \
        LOG_DEBUG << "  │  centroids   : (" \
                  << std::fixed << std::setprecision(2) << _p.centroid_ray_profile.centeroid_pair[0].x << ", " \
                  << _p.centroid_ray_profile.centeroid_pair[0].y << ")  <-->  (" \
                  << _p.centroid_ray_profile.centeroid_pair[1].x << ", " \
                  << _p.centroid_ray_profile.centeroid_pair[1].y << ")"; \
        \
        LOG_DEBUG << "  │  rays        : [" \
                  << std::setprecision(5) << _p.centroid_ray_profile.ray_pair[0][0] << ", " \
                  << _p.centroid_ray_profile.ray_pair[0][1] << ", " \
                  << _p.centroid_ray_profile.ray_pair[0][2] << "]  <-->  [" \
                  << _p.centroid_ray_profile.ray_pair[1][0] << ", " \
                  << _p.centroid_ray_profile.ray_pair[1][1] << ", " \
                  << _p.centroid_ray_profile.ray_pair[1][2] << "]"; \
        \
        if (_p.candidate_pair.empty()) { \
            LOG_DEBUG << "  │  candidates  : (none)"; \
        } else { \
            LOG_DEBUG << "  │  candidates  : " << _p.candidate_pair.size() << " pairs"; \
        } \
        LOG_DEBUG << "  └─────────────────────────────────"; \
    } \
} while(0)


extern CameraIntrinsic camera_intr_cfg;
extern lookupconfig lookup_cfg;
extern ProcessorConfig processor_cfg;

void processor_thread(
    std::atomic<std::shared_ptr<cv::Mat>>& latest_frame, 
    std::unordered_map<int, std::vector<StarPair>>& lookup, 
    std::atomic<bool>& running
)
{ 
    //  !frameready.load()continue -> get_frame_safe() -> frame_cpy_local.empty()continue -> preprocess_frame()
    double tsec = 0.0;
    auto next = std::chrono::steady_clock::now() + processor_cfg.period;

    
    
    //debug 
    bool flag_debug_png_export = false;  // to export only once 

    cv::Mat debug; 

    while (running.load())
    {
        auto frame = latest_frame.load(std::memory_order_acquire);
        cv::Mat preprocess_ed_frame;
        std::vector<cv::Point2d> centroids;
        std::vector<cv::Vec3d> ray;
        cv::Vec3d temp_ray; 
        CentroidRayPair pair;
        std::vector<StarHypothesis> hypothesis;
        AngSepProfile AngularSeparationProfile;
        
        std::this_thread::sleep_until(next);
        
        if (!frame) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        //Clone heavy -- keep track..
        cv::Mat frame_cpy_local = frame->clone();

        preprocess_ed_frame = ST::image::preprocess_frame(frame_cpy_local, tsec, processor_cfg.cv.blur_ksize, processor_cfg.cv.blur_sigma, processor_cfg.cv.threshold); //processing level -0
        centroids = ST::image::get_centroids(preprocess_ed_frame, tsec, processor_cfg.cv.area_max);
        
        

        //optimize the below for loops 
        //  --> loop one - find camera/body ray for each centroid


        // loop one - camera/body ray for each centroid
        // returns profile std::vector<cv::Vec3d> ray
        for (const auto& c : centroids) {
            
            temp_ray = ST::math::pixel_to_body_ray(c.x,c.y,camera_intr_cfg.fx,camera_intr_cfg.fy,camera_intr_cfg.cx,camera_intr_cfg.cy);
            if (ST::dbg::ray){
            LOG_DEBUG << "[Tracker] [Centroid] "<< "("<< c.x << ", " << c.y << ")";
            LOG_DEBUG << "[Tracker] [Centroid] [Ray] = ("<< temp_ray[0] << ", " << temp_ray[1] << ", " <<temp_ray[2]<< ")";
            }
            //ray.emplace_back(temp_ray);
            pair.centeroid_pair.emplace_back(c);
            pair.ray_pair.emplace_back(temp_ray);
        }
        


        
        //SAMPLE Angular separation implementation USE ONLY WITH 2 CENTROIDS ALONE !!!!!!! 
        
        //LOG_DEBUG << "[Tracker] [Centroid] [Ray] = ("<< ray[0] << "), (" << ray[1] << "), Angular separation -> " <<angle_between(ray[0],ray[1])<< "";
        
        for (size_t i = 0; i < pair.ray_pair.size(); i++){
            for (size_t j = i+1; j < pair.ray_pair.size(); j++){ 
                CentroidRayPair temp_profile;
                AngularSep_Profile_fields temp_fields;
                double theta = ST::math::AngularSeparationDeg(pair.ray_pair[i],pair.ray_pair[j]);  
                temp_profile.ray_pair.emplace_back(pair.ray_pair[i]);
                temp_profile.ray_pair.emplace_back(pair.ray_pair[j]);
                temp_profile.centeroid_pair.emplace_back(pair.centeroid_pair[i]);
                temp_profile.centeroid_pair.emplace_back(pair.centeroid_pair[j]);
                temp_profile.angular_separation = theta;
                temp_fields.centroid_ray_profile = temp_profile;
                AngularSeparationProfile.emplace_back(temp_fields);
                if (ST::dbg::ang_sep){  
                LOG_DEBUG << "[Tracker] [Centroid] [Ray] [Ang_Sep]["<<i<<","<<j<<"]= ("<< pair.ray_pair[i] << "), (" << pair.ray_pair[j] << "), Angular separation -> " <<theta<< "";
                }
            }
        }

        ST::identification::nStar::PopulateCandidatePairs(AngularSeparationProfile, lookup, lookup_cfg.tolerance, lookup_cfg.lookup_precision);
        if (ST::dbg::ang_profile){  
            PRINT_ANGULAR_SEP_PROFILE(AngularSeparationProfile);
        }

        hypothesis = ST::identification::nStar::vote_and_hypothesize(AngularSeparationProfile,centroids.size());

       for(const auto& h : hypothesis){
        LOG_DEBUG << h.centroid << " : "<< h.hip_id << "; ("<<h.vote_count<<")\n";
        }
        LOG_DEBUG<<"--------------------------------------\n";

        if (ST::dbg::img){
            cv::namedWindow("Unity Frame", cv::WINDOW_AUTOSIZE);
            cv::namedWindow("Preprocessed", cv::WINDOW_AUTOSIZE);
            cv::namedWindow("Centroid Debug", cv::WINDOW_AUTOSIZE);
            debug = ST::dbg::debug_pngexport_centroids(centroids,frame_cpy_local);
            cv::imshow("Unity Frame", frame_cpy_local);
            cv::imshow("Preprocessed", preprocess_ed_frame);        // or gray if you prefer
            cv::imshow("Centroid Debug", debug);
            // Required for window update
            cv::waitKey(1);   // 1 ms to allow GUI to refresh
        }


        AngularSeparationProfile.clear();

        //advance time with period for next wait_until(next)
        next += processor_cfg.period;
        tsec += std::chrono::duration_cast<std::chrono::duration<double>>(processor_cfg.period).count();
        //LOG_DEBUG<<"Processor Frame : "<<frame_cpy_local.cols<<" x "<<frame_cpy_local.rows<<"";
    }
    LOG_INFO << "[Tracker] exiting."; //debug exit
}