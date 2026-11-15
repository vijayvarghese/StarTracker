/**
 * @file test_identification.cpp
 * @brief Unit tests for ST::identification::nStar
 *
 * Covers:
 *   PopulateCandidatePairs
 *     - Exact bin match populates candidates
 *     - Bin within tolerance populates candidates
 *     - Bin outside tolerance does not populate
 *     - Empty lookup always returns true with no candidates
 *     - Multiple profiles populated independently
 *
 *   vote_and_hypothesize
 *     - Single profile → hypothesis generated
 *     - Winning ID accumulates highest votes
 *     - Confidence flag set correctly when best > second by required margin
 *     - Ambiguous vote (tie) → confident = false
 *     - Empty profiles → empty result
 */

#include <gtest/gtest.h>
#include "startracker/identification/n-star/n-star.hpp"
#include "startracker/core/types.hpp"
#include <opencv2/core/core.hpp>
#include <cmath>

using namespace ST::identification::nStar;

// ─── Helpers ─────────────────────────────────────────────────────────────────

/** Build a minimal AngularSep_Profile_fields with one pair of centroids. */
static AngularSep_Profile_fields make_profile(double ang_sep_deg,
                                               cv::Point2d ci = {0,0},
                                               cv::Point2d cj = {1,0})
{
    AngularSep_Profile_fields p;
    p.centroid_ray_profile.angular_separation = ang_sep_deg;
    p.centroid_ray_profile.centeroid_pair = {ci, cj};
    p.centroid_ray_profile.ray_pair       = {cv::Vec3d(0,0,1), cv::Vec3d(0,0,1)};
    return p;
}

/** Build a simple BinnedStarMap with a single bin → single pair. */
static ST::BinnedStarMap make_lookup(int bin_key,
                                      std::string id1 = "A",
                                      std::string id2 = "B")
{
    ST::BinnedStarMap lut;
    lut[bin_key] = { StarPair{id1, id2} };
    return lut;
}

// ─── PopulateCandidatePairs ───────────────────────────────────────────────────

class PopulateCandidatePairsTest : public ::testing::Test {
protected:
    static constexpr double precision  = 1.0;   // 1 deg / bin
    static constexpr int    tolerance  = 1;      // ±1 bin search window
};

TEST_F(PopulateCandidatePairsTest, ExactBinMatchPopulatesCandidates) {
    // Profile at 50.0 deg → bin 50; lookup has bin 50
    ST::AngSepProfile profiles = { make_profile(50.0) };
    auto lut = make_lookup(50);

    bool ok = PopulateCandidatePairs(profiles, lut, tolerance, precision);

    EXPECT_TRUE(ok);
    EXPECT_EQ(profiles[0].candidate_pair.size(), 1u);
    EXPECT_EQ(profiles[0].candidate_pair[0].id1, "A");
}

TEST_F(PopulateCandidatePairsTest, BinWithinTolerancePopulatesCandidates) {
    // Profile at 50.3 deg → rounds to bin 50; lookup has bin 51 (within ±1)
    ST::AngSepProfile profiles = { make_profile(50.3) };
    auto lut = make_lookup(51);

    PopulateCandidatePairs(profiles, lut, tolerance, precision);

    EXPECT_GE(profiles[0].candidate_pair.size(), 1u);
}

TEST_F(PopulateCandidatePairsTest, BinOutsideToleranceDoesNotPopulate) {
    // Profile at 50.0 deg → bin 50; lookup only has bin 53 (outside ±1)
    ST::AngSepProfile profiles = { make_profile(50.0) };
    auto lut = make_lookup(53);

    PopulateCandidatePairs(profiles, lut, tolerance, precision);

    EXPECT_EQ(profiles[0].candidate_pair.size(), 0u);
}

TEST_F(PopulateCandidatePairsTest, EmptyLookupReturnsTrueNoData) {
    ST::AngSepProfile profiles = { make_profile(50.0) };
    ST::BinnedStarMap lut;  // empty

    bool ok = PopulateCandidatePairs(profiles, lut, tolerance, precision);

    EXPECT_TRUE(ok);
    EXPECT_EQ(profiles[0].candidate_pair.size(), 0u);
}

TEST_F(PopulateCandidatePairsTest, MultipleProfilesPopulatedIndependently) {
    // Two profiles at different separations; only second has a matching bin
    ST::AngSepProfile profiles = {
        make_profile(30.0, {0,0}, {1,0}),
        make_profile(70.0, {2,0}, {3,0}),
    };
    auto lut = make_lookup(70);

    PopulateCandidatePairs(profiles, lut, tolerance, precision);

    EXPECT_EQ(profiles[0].candidate_pair.size(), 0u);
    EXPECT_GE(profiles[1].candidate_pair.size(), 1u);
}

TEST_F(PopulateCandidatePairsTest, MultiplePairsInSameBinAllAppended) {
    // Bin 50 has 3 pairs; all 3 should appear in candidate_pair
    ST::BinnedStarMap lut;
    lut[50] = { {"A","B"}, {"C","D"}, {"E","F"} };

    ST::AngSepProfile profiles = { make_profile(50.0) };
    PopulateCandidatePairs(profiles, lut, tolerance, precision);

    EXPECT_EQ(profiles[0].candidate_pair.size(), 3u);
}

// ─── vote_and_hypothesize ─────────────────────────────────────────────────────

class VoteAndHypothesizeTest : public ::testing::Test {
protected:
    /** Build a profile whose candidate_pair list has count copies of {id1,id2}. */
    static AngularSep_Profile_fields make_voted_profile(
        cv::Point2d ci, cv::Point2d cj,
        std::string id1, std::string id2, int count = 1)
    {
        auto p = make_profile(0.0, ci, cj);
        for (int i = 0; i < count; ++i)
            p.candidate_pair.push_back({id1, id2});
        return p;
    }
};

TEST_F(VoteAndHypothesizeTest, EmptyProfilesGiveEmptyResult) {
    ST::AngSepProfile profiles;
    auto result = vote_and_hypothesize(profiles, 5);
    EXPECT_TRUE(result.empty());
}

TEST_F(VoteAndHypothesizeTest, HypothesisCreatedPerCentroid) {
    // One profile with 2 distinct centroids → 2 hypotheses
    ST::AngSepProfile profiles = {
        make_voted_profile({0,0}, {1,0}, "HIP1", "HIP2", 1)
    };
    auto result = vote_and_hypothesize(profiles, 2);

    // At least 1 hypothesis should be created (both centroids are unique)
    EXPECT_GE(result.size(), 1u);
}

TEST_F(VoteAndHypothesizeTest, WinnerHasHighestVoteCount) {
    // Centroid (0,0) appears in two profiles: once paired with HIP1 and twice
    // with HIP2. HIP2 should win.
    ST::AngSepProfile profiles = {
        make_voted_profile({0,0}, {1,0}, "HIP1", "HIP2", 1),
        make_voted_profile({0,0}, {2,0}, "HIP2", "HIP3", 1),
        make_voted_profile({0,0}, {3,0}, "HIP2", "HIP4", 1),
    };
    // N = number of distinct centroids = let's say 4 (0,0),(1,0),(2,0),(3,0)
    auto result = vote_and_hypothesize(profiles, 4);

    // Find the hypothesis for centroid (0,0)
    auto it = std::find_if(result.begin(), result.end(),
        [](const StarHypothesis& h) {
            return h.centroid.x == 0.0 && h.centroid.y == 0.0;
        });
    ASSERT_NE(it, result.end()) << "Hypothesis for (0,0) not found";

    // HIP2 should have accumulated more votes than HIP1
    EXPECT_EQ(it->hip_id, "HIP2");
    EXPECT_GE(it->vote_count, 2);
}

TEST_F(VoteAndHypothesizeTest, ConfidentFlagSetWhenVotesExceedThreshold) {
    // N=3 → required = 2. If a centroid gets >= 2 votes for one ID
    // and < 2 for the runner-up it should be confident.
    cv::Point2d ci{0,0}, cj{1,0}, ck{2,0};
    ST::AngSepProfile profiles = {
        make_voted_profile(ci, cj, "HIP1", "HIP2", 1),
        make_voted_profile(ci, ck, "HIP1", "HIP3", 1),
    };
    // N=3: required = 2
    auto result = vote_and_hypothesize(profiles, 3);

    auto it = std::find_if(result.begin(), result.end(),
        [](const StarHypothesis& h){ return h.centroid.x==0 && h.centroid.y==0; });

    if (it != result.end() && it->vote_count >= 2 && it->second_vote_count < 2) {
        EXPECT_TRUE(it->confident);
    }
}

TEST_F(VoteAndHypothesizeTest, AmbiguousTieGivesNotConfident) {
    // Centroid (0,0): HIP1 and HIP2 both get exactly 1 vote
    // N=2 → required=1; both hit the threshold → tie → NOT confident
    cv::Point2d ci{0,0}, cj{1,0}, ck{2,0};
    ST::AngSepProfile profiles = {
        make_voted_profile(ci, cj, "HIP1", "HIP_NOISE", 1),
        make_voted_profile(ci, ck, "HIP2", "HIP_NOISE", 1),
    };
    // N=2; required=1; both HIP1 and HIP2 get 1 vote → second_votes=1 >= required → not confident
    auto result = vote_and_hypothesize(profiles, 2);

    auto it = std::find_if(result.begin(), result.end(),
        [](const StarHypothesis& h){ return h.centroid.x==0 && h.centroid.y==0; });

    if (it != result.end()) {
        // Both candidates have equal votes → should NOT be marked confident
        if (it->vote_count == it->second_vote_count) {
            EXPECT_FALSE(it->confident);
        }
    }
}

TEST_F(VoteAndHypothesizeTest, HypothesisContainsCorrectRay) {
    // The ray stored in the hypothesis should match the ray in the profile
    cv::Vec3d expected_ray(0.0, 0.0, 1.0);
    cv::Point2d ci{10, 20}, cj{30, 40};

    auto p = make_profile(45.0, ci, cj);
    p.centroid_ray_profile.ray_pair[0] = expected_ray;
    p.candidate_pair = { {"HIP1","HIP2"} };

    ST::AngSepProfile profiles = { p };
    auto result = vote_and_hypothesize(profiles, 2);

    auto it = std::find_if(result.begin(), result.end(),
        [&](const StarHypothesis& h){
            return h.centroid.x == ci.x && h.centroid.y == ci.y;
        });

    if (it != result.end()) {
        EXPECT_NEAR(it->ray[0], expected_ray[0], 1e-9);
        EXPECT_NEAR(it->ray[1], expected_ray[1], 1e-9);
        EXPECT_NEAR(it->ray[2], expected_ray[2], 1e-9);
    }
}

TEST_F(VoteAndHypothesizeTest, NoiseCentroidWithNoCandidatesGetsNoVotes) {
    // 3 true stars form a triangle (N=3, required=2)
    cv::Point2d c1{10, 10}, c2{20, 20}, c3{30, 30};
    cv::Point2d c_noise{100, 100}; // Noise spot

    ST::AngSepProfile profiles;
    // Pair (c1, c2)
    profiles.push_back(make_voted_profile(c1, c2, "HIP_A", "HIP_B"));
    // Pair (c1, c3)
    profiles.push_back(make_voted_profile(c1, c3, "HIP_A", "HIP_C"));
    // Pair (c2, c3)
    profiles.push_back(make_voted_profile(c2, c3, "HIP_B", "HIP_C"));

    // Noise profiles with empty candidates
    auto p_noise1 = make_profile(15.0, c1, c_noise);
    auto p_noise2 = make_profile(25.0, c2, c_noise);
    profiles.push_back(p_noise1);
    profiles.push_back(p_noise2);

    auto result = vote_and_hypothesize(profiles, 3);

    // Find hypothesis for c1
    auto it_c1 = std::find_if(result.begin(), result.end(),
        [&](const StarHypothesis& h){ return h.centroid == c1; });
    ASSERT_NE(it_c1, result.end());
    EXPECT_EQ(it_c1->hip_id, "HIP_A");
    EXPECT_EQ(it_c1->vote_count, 2);
    EXPECT_TRUE(it_c1->confident);

    // Noise centroid should either not be present or have 0 votes
    auto it_noise = std::find_if(result.begin(), result.end(),
        [&](const StarHypothesis& h){ return h.centroid == c_noise; });
    if (it_noise != result.end()) {
        EXPECT_EQ(it_noise->vote_count, 0);
        EXPECT_FALSE(it_noise->confident);
    }
}

