#include <opencv2/opencv.hpp>
#include <unordered_map>
#include <unordered_set>
#include <fstream>


#include "startracker/identification/n-star/n-star.hpp"
#include "startracker/core/types.hpp"
#include "startracker/core/logger.hpp" 
#include "config.hpp"

 

namespace ST::identification::nStar{

bool PopulateCandidatePairs(AngSepProfile& profiles, const std::unordered_map<int,std::vector<StarPair>>& lookup_, const double& tolerance, const double& precision){

    for (auto& profile : profiles){
        int center_bin = (int) std::round(profile.centroid_ray_profile.angular_separation / precision);
        for(int offset = -tolerance; offset <= tolerance; ++offset){
            int bin = center_bin + offset;
            auto it = lookup_.find(bin);
            if(it != lookup_.end()){
                const auto& can = it->second;
                profile.candidate_pair.insert(profile.candidate_pair.end(), can.begin(),can.end());
            }
        }
    }
    return true;
}


std::vector<StarHypothesis> vote_and_hypothesize(
    const AngSepProfile& profiles,
    int N)  // number of detected centroids — 9 in your case
{
    const int required = N - 1;  // 8

    // star_votes[pixel_coord][hip_id] = count
    std::map<cv::Point2d,
             std::unordered_map<std::string, int>,
             Point2dCmp> star_votes;

    // also keep profile reverse map pixel → ray
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
        h.hip_id_second_best= second_id; //for later debug, remove if not implimented !! along with the second_id ref on top. 
        

        hypothesis.push_back(h);
    }

    // summary line


    return hypothesis;
}


} // namespace ST::identification::nStar
