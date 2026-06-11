#pragma once
#include <atomic>
#include <opencv2/core/core.hpp>
#include <string>
#include <unordered_map>
#include <vector>

struct StarPair {
  std::string id1;
  std::string id2;
};

struct CentroidRayPair {
  double angular_separation;
  std::vector<cv::Point2d> centeroid_pair;
  std::vector<cv::Vec3d> ray_pair;
};

// Creating a struct to hold CentroidRayPair and the bin (Candidate pair list)
struct AngularSep_Profile_fields {
  CentroidRayPair centroid_ray_profile;
  std::vector<StarPair> candidate_pair;
};

// comparator so cv::Point2d can be used as a std::map key
struct Point2dCmp {
  bool operator()(const cv::Point2d &a, const cv::Point2d &b) const {
    if (a.x != b.x)
      return a.x < b.x;
    return a.y < b.y;
  }
};

// result per centroid
struct StarHypothesis {
  cv::Point2d centroid;  // the pixel coordinate
  cv::Vec3d ray;         // its body-frame ray
  std::string hip_id;    // winning HIP ID
  int vote_count;        // how many votes it got
  int second_vote_count; // runner-up votes (for ambiguity check)
  bool confident;        // true if vote_count == N-1 and no tie
  std::string hip_id_second_best;
};

enum class ConfigInitStatus { Json = 1, FallBack = 2, Error = 3 };

enum class BinLoadStatus { Ok = 0, Error = 1 };

using AngSepProfile = std::vector<AngularSep_Profile_fields>;
// using FrameAtom = std::atomic<std::shared_ptr<cv::Mat>>;

std::atomic<bool> HAL_isTcpEnabled = true;