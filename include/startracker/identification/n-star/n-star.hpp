#pragma once
#include "startracker/core/types.hpp"
#include <opencv2/core/core.hpp>

namespace ST::identification::nStar {

/**
 * @brief Populate the the cadidate pair field at the angular seperation
 * profile, From the lookup table bin.
 * @param profiles Reference to vector of Angular seperation fields.
 * @param lookup_ reference to the lookup table.
 * @return true/false
 */
bool PopulateCandidatePairs(
    ST::AngSepProfile &profiles,
    const std::unordered_map<int, std::vector<StarPair>> &lookup_,
    const double &tolerance, const double &precision);

/**
 * @brief Generate star identity hypotheses using centroid vote aggregation.
 *
 * This function performs a voting-based star identification pass over
 * angular separation profiles.
 *
 * For each angular profile:
 *  - Candidate star IDs are collected from lookup matches.
 *  - Votes are accumulated independently for each detected centroid.
 *  - Each centroid maintains a histogram of candidate HIP IDs.
 *
 * After all profiles are processed:
 *  - The highest-voted HIP ID is selected per centroid.
 *  - Confidence is determined using vote thresholds.
 *  - A StarHypothesis object is generated for each centroid.
 *
 * Voting logic:
 *  - Each unique HIP ID contributes at most one vote per profile
 *    per centroid.
 *  - A hypothesis is considered confident when:
 *
 *        best_votes >= (N - 1)
 *
 *    AND
 *
 *        second_best_votes < (N - 1)
 *
 * where N is the number of detected centroids.
 *
 * @param profiles
 * Angular separation profiles containing:
 *  - centroid/ray pairs
 *  - candidate star-pair lookup matches
 *
 * @param N
 * Number of detected centroids in the frame.
 * Used to compute required confidence vote threshold:
 *
 *      required_votes = N - 1
 *
 * @return std::vector<StarHypothesis>
 * Vector of generated star hypotheses, one per centroid.
 * Each hypothesis contains:
 *  - centroid location
 *  - associated ray
 *  - best HIP ID candidate
 *  - vote statistics
 *  - confidence state
 */
std::vector<StarHypothesis> vote_and_hypothesize(const AngSepProfile &profiles,
                                                 int N);

} // namespace ST::identification::nStar
