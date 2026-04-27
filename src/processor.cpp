// Processing Thread
#include<iostream>
#include<opencv2/opencv.hpp>
#include<thread>
#include<chrono>
#include<atomic>
#include<mutex>
#include <unordered_map>
#include <unordered_set>
#include "globals.h"
#include "processor.hpp"
#include "config.hpp"
#include "logger.hpp"


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
//macro End

//            for (size_t _ci = 0; _ci < _p.candidate_pair.size(); _ci++) { \
//                const char* _pfx = (_ci == _p.candidate_pair.size()-1) ? "  │      └─ " : "  │      ├─ "; \
//                LOG_DEBUG << _pfx << "[" << _ci << "]  " \
//                          << _p.candidate_pair[_ci].id1 << "  <-->  " \
//                          << _p.candidate_pair[_ci].id2; \
//            } \


//Struct to store the centroid and ray infor for an agular seperation 

struct CentroidRayPair{
    double angular_separation;
    std::vector<cv::Point2d> centeroid_pair;
    std::vector<cv::Vec3d> ray_pair;
};


// Creating a struct to hold CentroidRayPair and the bin (Candidate pair list)
struct AngularSep_Profile_fields{
    CentroidRayPair centroid_ray_profile;
    std::vector<StarPair> candidate_pair; 
};


std::vector<AngularSep_Profile_fields> AngularSeparationProfile;




cv::Mat get_frame_safe(){ //critical section (return the clone of the latestframe as cv::Mat) implementation @33
    std::lock_guard<std::mutex> local_cpy_lock(M_latestframe);
    if(latestframe.empty()) return cv::Mat();
    return latestframe.clone();
}

cv::Mat preprocess_frame(cv::Mat &frame, double &ts){ // preprocessing each frame
    cv::Mat gray, blurred, bw;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY); //gray scale
    cv::GaussianBlur(gray, blurred, cv::Size(processor_cfg.cv.blur_ksize,processor_cfg.cv.blur_ksize), processor_cfg.cv.blur_sigma); //induced blur
    cv::threshold(blurred, bw, processor_cfg.cv.threshold, 255, cv::THRESH_BINARY); //any dot above 200 gets full brightness 255 rest full black.
    int bright = cv::countNonZero(bw); // count white dots
    if (processor_centeroid_debug){
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
    if(processor_centeroid_debug){
    LOG_DEBUG << "[Tracker] [Centroid] t=" << ts
              << " | stars=" << temp_centroid.size()
              << " | size=" << BW.cols << "x" << BW.rows;
    }

    // just to print the centroids on console....
    if (processor_centeroid_debug){
    int idx = 0;
    for (const auto& c : temp_centroid) {
    LOG_DEBUG << "[Tracker] [Centroid]"<< "Centroid[" << idx++ << "] = ("
              << c.x << ", " << c.y << ")";
    }
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
// Assumption - body frame is aligned with the camera frame... 
//              ->  body frame z axis is same as the camera z axis (focal axis, perpendicular to the image plane.) 

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


//Hypothesis


//Pyramid method
//bin lookup 
bool look_it_up(std::vector<AngularSep_Profile_fields>& profiles, const std::unordered_map<int,std::vector<StarPair>>& lookup_){

    for (auto& a : profiles){
        int center_bin = (int) std::round(a.centroid_ray_profile.angular_separation / lookup_cfg.lookup_precision);
        for(int offset = -lookup_cfg.tolerance; offset <= lookup_cfg.tolerance; ++offset){
            int bin = center_bin + offset;
            auto it = lookup_.find(bin);
            if(it != lookup_.end()){
                const auto& can = it->second;
                a.candidate_pair.insert(a.candidate_pair.end(), can.begin(),can.end());
            }
        }
    }
    return true;
}


//finding base triangle candidates
//For now runnign voting for all centeroids 

// comparator so cv::Point2d can be used as a std::map key
struct Point2dCmp {
    bool operator()(const cv::Point2d& a, const cv::Point2d& b) const {
        if (a.x != b.x) return a.x < b.x;
        return a.y < b.y;
    }
};

// result per centroid
struct StarHypothesis {
    cv::Point2d centroid;          // the pixel coordinate
    cv::Vec3d   ray;               // its body-frame ray
    std::string hip_id;            // winning HIP ID
    int         vote_count;        // how many votes it got
    int         second_vote_count; // runner-up votes (for ambiguity check)
    bool        confident;         // true if vote_count == N-1 and no tie
};

std::vector<StarHypothesis> vote_and_hypothesize(
    const std::vector<AngularSep_Profile_fields>& profiles,
    int N)  // number of detected centroids — 9 in your case
{
    const int required = N - 1;  // 8

    // star_votes[pixel_coord][hip_id] = count
    std::map<cv::Point2d,
             std::unordered_map<std::string, int>,
             Point2dCmp> star_votes;

    // also keep a reverse map pixel → ray
    // so we can populate StarHypothesis.ray without re-searching
    std::map<cv::Point2d, cv::Vec3d, Point2dCmp> centroid_ray_map;

    // ── cast votes across all 36 profiles ──────────────────────────
    for (const auto& prof : profiles) {
    const cv::Point2d& ci = prof.centroid_ray_profile.centeroid_pair[0];
    const cv::Point2d& cj = prof.centroid_ray_profile.centeroid_pair[1];

    // collect unique IDs seen in THIS profile for ci and cj separately
    std::unordered_set<std::string> seen_ci;
    std::unordered_set<std::string> seen_cj;

    for (const auto& sp : prof.candidate_pair) {
        // ci could be either id1 or id2
        seen_ci.insert(sp.id1);
        seen_ci.insert(sp.id2);

        // cj could be either id1 or id2
        seen_cj.insert(sp.id1);
        seen_cj.insert(sp.id2);
    }

    // now commit exactly ONE vote per unique ID per centroid per profile
    for (const auto& id : seen_ci) star_votes[ci][id]++;
    for (const auto& id : seen_cj) star_votes[cj][id]++;
    }
//debug
{
auto& last_bucket = star_votes.rbegin()->second;
const auto& last_pixel = star_votes.rbegin()->first;

// collect and sort
std::vector<std::pair<int,std::string>> ranked;
for (auto& [id, count] : last_bucket)
    ranked.push_back({count, id});
std::sort(ranked.rbegin(), ranked.rend());

LOG_DEBUG << "[Diag] top 5 votes for ("
          << last_pixel.x << ", " << last_pixel.y << "):";
for (int k = 0; k < std::min(5, (int)ranked.size()); k++)
    LOG_DEBUG << "[Diag]   " << ranked[k].second
              << "  count=" << ranked[k].first;

}
    // ── find winner per centroid ────────────────────────────────────
    std::vector<StarHypothesis> hypothesis;
    hypothesis.reserve(star_votes.size());

    for (auto& [pixel, id_map] : star_votes) {

        std::string best_id, second_id;
        int best_votes   = 0;
        int second_votes = 0;

        for (const auto& [id, count] : id_map) {
            if (count > best_votes) {
                second_votes = best_votes;
                second_id    = best_id;
                best_votes   = count;
                best_id      = id;
            } else if (count > second_votes) {
                second_votes = count;
                second_id    = id;
            }
        }

        bool confident = (best_votes >= required) && (second_votes < required);

        StarHypothesis h;
        h.centroid          = pixel;
        h.ray               = centroid_ray_map[pixel];
        h.hip_id            = best_id;
        h.vote_count        = best_votes;
        h.second_vote_count = second_votes;
        h.confident         = confident;

        hypothesis.push_back(h);

        // log each result
        if (confident) {
            LOG_INFO << "[Vote] (" << std::fixed << std::setprecision(1)
                     << pixel.x << ", " << pixel.y << ")"
                     << "  =>  " << best_id
                     << "  votes=" << best_votes
                     << "  2nd="   << second_votes;
        } else {
            LOG_WARN << "[Vote] (" << std::fixed << std::setprecision(1)
                     << pixel.x << ", " << pixel.y << ")"
                     << "  AMBIGUOUS"
                     << "  best=" << best_votes
                     << "  2nd="  << second_votes
                     << "  required=" << required;
        }
    }

    // summary line
    int confident_count = 0;
    for (const auto& h : hypothesis)
        if (h.confident) confident_count++;

    LOG_INFO << "[Vote] " << confident_count << "/" << N
             << " centroids identified with confidence";

    return hypothesis;
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
        CentroidRayPair pair;
        
        std::this_thread::sleep_until(next);

        if(!frameready.load()){
            LOG_WARN << "[Tracker] No frame ready yet, skipping tick.";
            std::this_thread::sleep_for(std::chrono::milliseconds(2));//avoid busy-waiting need to change it to condition variable
            goto advance;
        }

        
        { //Critical Section safe function call @16
            frame_cpy_local = get_frame_safe();
        }

        if (frame_cpy_local.empty()) {
            LOG_WARN << "[Tracker] Frame was empty after grabbing, skipping.";
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
            
            temp_ray = pixel_to_body_ray(c.x,c.y,camera_intr_cfg.fx,camera_intr_cfg.fy,camera_intr_cfg.cx,camera_intr_cfg.cy);
            if (processor_ray_debug){
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
                //theta_vec.emplace_back(angle_between(ray[i],ray[j+1])); 
                CentroidRayPair temp_profile;
                AngularSep_Profile_fields temp_fields;
                double theta = angle_between(pair.ray_pair[i],pair.ray_pair[j]);  
                temp_profile.ray_pair.emplace_back(pair.ray_pair[i]);
                temp_profile.ray_pair.emplace_back(pair.ray_pair[j]);
                temp_profile.centeroid_pair.emplace_back(pair.centeroid_pair[i]);
                temp_profile.centeroid_pair.emplace_back(pair.centeroid_pair[j]);
                temp_profile.angular_separation = theta;
                temp_fields.centroid_ray_profile = temp_profile;
                AngularSeparationProfile.emplace_back(temp_fields);
                //if (processor_AngProfile_debug){  
                //PRINT_ANGULAR_SEP_PROFILE(AngularSeparationProfile);
                //}
                if (processor_angSep_debug){  
                LOG_DEBUG << "[Tracker] [Centroid] [Ray] [Ang_Sep]["<<i<<","<<j<<"]= ("<< pair.ray_pair[i] << "), (" << pair.ray_pair[j] << "), Angular separation -> " <<theta<< "";
                }
            }
        }
        //debug
LOG_DEBUG << "[Diag] profile count before lookup: "
          << AngularSeparationProfile.size();
// should always print 36, never 72/108/144...

        look_it_up(AngularSeparationProfile, lookup);
        if (processor_AngProfile_debug){  
            PRINT_ANGULAR_SEP_PROFILE(AngularSeparationProfile);
        }

    

        vote_and_hypothesize(AngularSeparationProfile,centroids.size());


        if (processor_img_debug){
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


        AngularSeparationProfile.clear();
        advance:
        //advance time with period for next wait_until(next)
        next += processor_cfg.period;
        tsec += std::chrono::duration_cast<std::chrono::duration<double>>(processor_cfg.period).count();
        //LOG_DEBUG<<"Processor Frame : "<<frame_cpy_local.cols<<" x "<<frame_cpy_local.rows<<"";
    }
    LOG_INFO << "[Tracker] exiting."; //debug exit
}