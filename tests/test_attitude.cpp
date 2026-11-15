/**
 * @file test_attitude.cpp
 * @brief Unit tests for ST::attitude::solve_quest (QUEST attitude solver).
 *
 * Covers:
 *   - Degenerate input (< 3 matched stars) → valid=false
 *   - Identity rotation (body rays == reference rays) → q ≈ identity, rms ≈ 0
 *   - Known rotation: body = R·reference → recovered q ≈ R, rms < threshold
 *   - lambda_max bounds: must be in (0, 1] for well-conditioned problem
 *   - Zero-ray guard: stars with zero body_ray are silently skipped
 *   - Missing catalog ID guard: stars not in catalog are silently skipped
 */

#include <gtest/gtest.h>
#include "startracker/attitude/quest.hpp"
#include "startracker/core/types.hpp"

#include <Eigen/Geometry>
#include <cmath>
#include <numbers>
#include <opencv2/core.hpp>

// ─── Helpers ─────────────────────────────────────────────────────────────────

static constexpr double kQuat  = 1e-6;  // tolerance for quaternion components
static constexpr double kRms   = 0.05;  // acceptable RMS (deg) for a perfect synthetic input
static constexpr double kLamba = 1e-6;

/** Build a StarProfileMap from {hip_id, ra_deg, dec_deg} triples. */
static ST::StarProfileMap make_catalog(
    std::initializer_list<std::tuple<std::string,double,double>> entries)
{
    ST::StarProfileMap cat;
    for (auto& [id, ra_deg, dec_deg] : entries) {
        cat[id].ra_rad  = ra_deg  * std::numbers::pi / 180.0;
        cat[id].dec_rad = dec_deg * std::numbers::pi / 180.0;
        cat[id].mag     = 3.0;
    }
    return cat;
}

/** Convert RA/Dec (radians) to ECI unit vector (same formula used internally). */
static Eigen::Vector3d ra_dec_to_eci(double ra_rad, double dec_rad) {
    return Eigen::Vector3d(
        std::cos(dec_rad) * std::cos(ra_rad),
        std::cos(dec_rad) * std::sin(ra_rad),
        std::sin(dec_rad)
    ).normalized();
}

// ─── Fixture ──────────────────────────────────────────────────────────────────

class QuestTest : public ::testing::Test {
protected:
    // Small catalog: four well-separated stars
    ST::StarProfileMap catalog = make_catalog({
        {"HIP1",   0.0,   0.0},
        {"HIP2",  90.0,   0.0},
        {"HIP3",   0.0,  90.0},
        {"HIP4", 180.0,  45.0},
        {"HIP5",  45.0, -30.0},
    });
};

// ─── Test: too few matched stars ──────────────────────────────────────────────

TEST_F(QuestTest, TooFewMatchedStarsReturnInvalid) {
    std::vector<ST::attitude::MatchedStar> stars = {
        { cv::Vec3d(0, 0, 1), "HIP1" },
        { cv::Vec3d(1, 0, 0), "HIP2" },
    };
    auto result = ST::attitude::solve_quest(stars, catalog);
    EXPECT_FALSE(result.valid);
    EXPECT_LE(result.n_stars, 2);
}

TEST_F(QuestTest, EmptyMatchedStarsReturnInvalid) {
    auto result = ST::attitude::solve_quest({}, catalog);
    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.n_stars, 0);
}

// ─── Test: zero-ray guard ────────────────────────────────────────────────────

TEST_F(QuestTest, ZeroRayStarsAreSkipped) {
    // Provide 3 valid + 1 zero-ray; result should still have n_stars == 3
    std::vector<ST::attitude::MatchedStar> stars = {
        { cv::Vec3d(0, 0, 0), "HIP1" },   // zero ray → skipped
        { cv::Vec3d(1, 0, 0), "HIP2" },
        { cv::Vec3d(0, 1, 0), "HIP3" },
        { cv::Vec3d(0, 0, 1), "HIP4" },
    };
    auto result = ST::attitude::solve_quest(stars, catalog);
    EXPECT_EQ(result.n_stars, 3);
}

// ─── Test: unknown HIP ID guard ──────────────────────────────────────────────

TEST_F(QuestTest, UnknownHipIdIsSkipped) {
    std::vector<ST::attitude::MatchedStar> stars = {
        { cv::Vec3d(1, 0, 0), "UNKNOWN_99999" },
        { cv::Vec3d(0, 1, 0), "HIP2" },
        { cv::Vec3d(0, 0, 1), "HIP3" },
        { cv::Vec3d(1, 1, 0), "HIP4" },
    };
    // Only 3 valid pairs → solver still runs
    auto result = ST::attitude::solve_quest(stars, catalog);
    EXPECT_LE(result.n_stars, 3);
}

// ─── Test: identity rotation ─────────────────────────────────────────────────
//
// When the body rays ARE the reference ECI vectors (identity rotation R=I)
// the solver should return q ≈ identity and rms ≈ 0.
//
// Convention: pixel_to_body_ray gives Z+ toward the star (same sense as the
// ECI reference vector). For identity attitude, body frame = ECI frame, so
// body_ray = r̂ directly. No negation is applied inside solve_quest.

TEST_F(QuestTest, IdentityRotationRecovered) {
    // For identity rotation (R = I): body_ray = r̂  (both point toward the star).
    std::vector<ST::attitude::MatchedStar> stars;
    for (auto& [id, prof] : catalog) {
        Eigen::Vector3d r = ra_dec_to_eci(prof.ra_rad, prof.dec_rad);
        stars.push_back({ cv::Vec3d(r.x(), r.y(), r.z()), id });
    }

    auto result = ST::attitude::solve_quest(stars, catalog);

    EXPECT_TRUE(result.valid) << "Identity solve should be valid";
    EXPECT_GE(result.lambda_max, 0.5);

    // Quaternion should be close to identity (w≈1 due to positive-hemisphere fix)
    Eigen::Quaterniond q = result.q;
    double w = std::abs(q.w());
    EXPECT_NEAR(w, 1.0, 1e-4) << "Identity rotation: w component should be ~1";

    EXPECT_NEAR(result.rms_deg, 0.0, kRms);
}

// ─── Test: lambda_max validity ────────────────────────────────────────────────

TEST_F(QuestTest, LambdaMaxBoundedByOne) {
    std::vector<ST::attitude::MatchedStar> stars;
    for (auto& [id, prof] : catalog) {
        Eigen::Vector3d r = ra_dec_to_eci(prof.ra_rad, prof.dec_rad);
        stars.push_back({ cv::Vec3d(r.x(), r.y(), r.z()), id });
    }
    auto result = ST::attitude::solve_quest(stars, catalog);
    // lambda_max for QUEST is always ≤ 1 (it equals 1 for a perfect fit)
    EXPECT_LE(result.lambda_max, 1.0 + kLamba);
    EXPECT_GE(result.lambda_max, 0.0);
}

// ─── Test: AttitudeResult fields ─────────────────────────────────────────────

TEST_F(QuestTest, AttitudeResultQuaternionIsNormalized) {
    std::vector<ST::attitude::MatchedStar> stars;
    for (auto& [id, prof] : catalog) {
        Eigen::Vector3d r = ra_dec_to_eci(prof.ra_rad, prof.dec_rad);
        stars.push_back({ cv::Vec3d(r.x(), r.y(), r.z()), id });
    }
    auto result = ST::attitude::solve_quest(stars, catalog);
    if (result.valid) {
        double norm = result.q.norm();
        EXPECT_NEAR(norm, 1.0, 1e-9);
    }
}

// ─── Test: Arbitrary 3D Rotation Recovery ───────────────────────────────────

TEST_F(QuestTest, ArbitraryRotationRecovered) {
    // Rotation of 37 degrees around unit axis [1, 2, 2]/3
    Eigen::Vector3d axis = Eigen::Vector3d(1.0, 2.0, 2.0).normalized();
    double angle = 37.0 * std::numbers::pi / 180.0;
    Eigen::Quaterniond q_true(Eigen::AngleAxisd(angle, axis));
    if (q_true.w() < 0.0) q_true.coeffs() = -q_true.coeffs();

    Eigen::Matrix3d R_true = q_true.toRotationMatrix();

    std::vector<ST::attitude::MatchedStar> stars;
    for (auto& [id, prof] : catalog) {
        Eigen::Vector3d r = ra_dec_to_eci(prof.ra_rad, prof.dec_rad);
        Eigen::Vector3d b = R_true * r;
        stars.push_back({ cv::Vec3d(b.x(), b.y(), b.z()), id });
    }

    auto result = ST::attitude::solve_quest(stars, catalog);

    EXPECT_TRUE(result.valid);
    EXPECT_NEAR(result.rms_deg, 0.0, 0.01);
    EXPECT_GE(result.lambda_max, 0.99);

    // Quaternion error: |q · q_true| should be ~ 1
    double dot = std::abs(result.q.dot(q_true));
    EXPECT_NEAR(dot, 1.0, 1e-4);
}

// ─── Test: Outlier Rejection via MAD ──────────────────────────────────────────

TEST_F(QuestTest, OutlierRejectionPrunesMisidentifiedStar) {
    // Rotate all catalog stars by a known rotation
    Eigen::Vector3d axis = Eigen::Vector3d(0.0, 1.0, 0.0);
    double angle = 25.0 * std::numbers::pi / 180.0;
    Eigen::Quaterniond q_true(Eigen::AngleAxisd(angle, axis));
    if (q_true.w() < 0.0) q_true.coeffs() = -q_true.coeffs();

    Eigen::Matrix3d R_true = q_true.toRotationMatrix();

    std::vector<ST::attitude::MatchedStar> stars;
    for (auto& [id, prof] : catalog) {
        Eigen::Vector3d r = ra_dec_to_eci(prof.ra_rad, prof.dec_rad);
        Eigen::Vector3d b = R_true * r;
        stars.push_back({ cv::Vec3d(b.x(), b.y(), b.z()), id });
    }

    // Corrupt one star with a completely bogus body ray (outlier)
    ASSERT_GE(stars.size(), 4u);
    stars[0].body_ray = cv::Vec3d(-stars[0].body_ray[0], -stars[0].body_ray[1], stars[0].body_ray[2]);

    auto result = ST::attitude::solve_quest(stars, catalog);

    // Should successfully reject the corrupted star and converge on remaining
    EXPECT_TRUE(result.valid);
    EXPECT_EQ(result.n_stars, static_cast<int>(stars.size() - 1));
    EXPECT_LT(result.rms_deg, 0.1);

    double dot = std::abs(result.q.dot(q_true));
    EXPECT_NEAR(dot, 1.0, 1e-3);
}

// ─── Test: Robustness with Measurement Noise ─────────────────────────────────

TEST_F(QuestTest, MeasurementNoiseDegradesGracefully) {
    Eigen::Quaterniond q_true = Eigen::Quaterniond::Identity();
    std::vector<ST::attitude::MatchedStar> stars;

    // Small deterministic perturbations (~0.02 deg)
    double perturb = 0.02 * std::numbers::pi / 180.0;
    int idx = 0;
    for (auto& [id, prof] : catalog) {
        Eigen::Vector3d r = ra_dec_to_eci(prof.ra_rad, prof.dec_rad);
        // Add small offset to body ray
        Eigen::Vector3d b = r + Eigen::Vector3d(
            (idx % 2 == 0 ? 1 : -1) * perturb,
            (idx % 3 == 0 ? 1 : -1) * perturb,
            0.0
        );
        b.normalize();
        stars.push_back({ cv::Vec3d(b.x(), b.y(), b.z()), id });
        idx++;
    }

    auto result = ST::attitude::solve_quest(stars, catalog);

    EXPECT_TRUE(result.valid);
    EXPECT_GT(result.lambda_max, 0.95);
    EXPECT_LT(result.rms_deg, 0.2); // Within expected noise envelope
}

