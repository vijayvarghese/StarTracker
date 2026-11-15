/**
 * @file test_math.cpp
 * @brief Unit tests for ST::math geometry functions.
 *
 * Covers:
 *   - pixel_to_body_ray  : pin-hole camera projection + normalization
 *   - AngularSeparationDeg : angle between two 3-D unit vectors
 */

#include <gtest/gtest.h>
#include "startracker/math/geometry.hpp"
#include <opencv2/core/core.hpp>
#include <cmath>

// ─── Helpers ─────────────────────────────────────────────────────────────────

static constexpr double kEps = 1e-9;   // near-exact tolerance
static constexpr double kDegEps = 1e-6; // ~1 µdeg for angle comparisons

// ─── pixel_to_body_ray ────────────────────────────────────────────────────────

class PixelToBodyRayTest : public ::testing::Test {
protected:
    // Typical camera intrinsics (arbitrary but realistic)
    const double fx = 800.0, fy = 800.0;
    const double cx = 320.0, cy = 240.0;
};

TEST_F(PixelToBodyRayTest, PrincipalPointGivesPositiveZAxis) {
    // A pixel exactly at (cx, cy) maps to the optical axis → (0,0,1) normalised
    cv::Vec3d r = ST::math::pixel_to_body_ray(cx, cy, fx, fy, cx, cy);

    EXPECT_NEAR(r[0], 0.0, kEps);
    EXPECT_NEAR(r[1], 0.0, kEps);
    EXPECT_NEAR(r[2], 1.0, kEps);
}

TEST_F(PixelToBodyRayTest, OutputIsUnitVector) {
    // For any pixel the result must be a unit vector
    const std::vector<std::pair<double,double>> pixels = {
        {0.0, 0.0}, {100.0, 50.0}, {640.0, 480.0}, {320.5, 240.5}
    };
    for (auto [u, v] : pixels) {
        cv::Vec3d r = ST::math::pixel_to_body_ray(u, v, fx, fy, cx, cy);
        double len = cv::norm(r);
        EXPECT_NEAR(len, 1.0, kEps)
            << "Failed for pixel (" << u << "," << v << ")";
    }
}

TEST_F(PixelToBodyRayTest, PixelRightOfCenterHasPositiveX) {
    // u > cx → x-component of unnormalised ray is positive
    cv::Vec3d r = ST::math::pixel_to_body_ray(cx + 100.0, cy, fx, fy, cx, cy);
    EXPECT_GT(r[0], 0.0);
    EXPECT_NEAR(r[1], 0.0, kEps);
}

TEST_F(PixelToBodyRayTest, PixelBelowCenterHasPositiveY) {
    // v > cy → y-component positive (image-space y grows downward)
    cv::Vec3d r = ST::math::pixel_to_body_ray(cx, cy + 100.0, fx, fy, cx, cy);
    EXPECT_NEAR(r[0], 0.0, kEps);
    EXPECT_GT(r[1], 0.0);
}

TEST_F(PixelToBodyRayTest, ZComponentAlwaysPositive) {
    // z is always 1 / |r| before normalisation, so result[2] > 0 always
    cv::Vec3d r = ST::math::pixel_to_body_ray(0.0, 0.0, fx, fy, cx, cy);
    EXPECT_GT(r[2], 0.0);
}

TEST_F(PixelToBodyRayTest, SymmetricPixelsMirroredCorrectly) {
    // Pixel left of centre → negative x; right of centre → same magnitude, positive x
    double u_off = 80.0;
    cv::Vec3d left  = ST::math::pixel_to_body_ray(cx - u_off, cy, fx, fy, cx, cy);
    cv::Vec3d right = ST::math::pixel_to_body_ray(cx + u_off, cy, fx, fy, cx, cy);

    EXPECT_NEAR(left[0], -right[0], kEps);
    EXPECT_NEAR(left[1],  right[1], kEps);
    EXPECT_NEAR(left[2],  right[2], kEps);
}

TEST_F(PixelToBodyRayTest, DifferentFocalLengthsAffectXYRatio) {
    // With fx=400, fy=800 the x deflection for a given du is larger than y
    double fy_half = 400.0;
    cv::Vec3d r = ST::math::pixel_to_body_ray(cx + 80.0, cy + 80.0, fx, fy_half, cx, cy);
    // unnorm x = 80/800 = 0.1, unnorm y = 80/400 = 0.2  → |y| > |x|
    EXPECT_GT(std::abs(r[1]), std::abs(r[0]));
}

// ─── AngularSeparationDeg ────────────────────────────────────────────────────

TEST(AngularSepTest, IdenticalVectorsGiveZeroDeg) {
    cv::Vec3d v(1.0, 0.0, 0.0);
    double angle = ST::math::AngularSeparationDeg(v, v);
    EXPECT_NEAR(angle, 0.0, kDegEps);
}

TEST(AngularSepTest, OppositeVectorsGive180Deg) {
    cv::Vec3d a(1.0, 0.0, 0.0);
    cv::Vec3d b(-1.0, 0.0, 0.0);
    double angle = ST::math::AngularSeparationDeg(a, b);
    EXPECT_NEAR(angle, 180.0, kDegEps);
}

TEST(AngularSepTest, OrthogonalVectorsGive90Deg) {
    cv::Vec3d a(1.0, 0.0, 0.0);
    cv::Vec3d b(0.0, 1.0, 0.0);
    double angle = ST::math::AngularSeparationDeg(a, b);
    EXPECT_NEAR(angle, 90.0, kDegEps);
}

TEST(AngularSepTest, Commutative) {
    cv::Vec3d a = cv::normalize(cv::Vec3d(1.0, 2.0, 3.0));
    cv::Vec3d b = cv::normalize(cv::Vec3d(-2.0, 1.0, 0.5));
    double ab = ST::math::AngularSeparationDeg(a, b);
    double ba = ST::math::AngularSeparationDeg(b, a);
    EXPECT_NEAR(ab, ba, kDegEps);
}

TEST(AngularSepTest, KnownAngle45Deg) {
    // Two vectors 45° apart in the XZ plane
    cv::Vec3d a(1.0, 0.0, 0.0);
    // 45° from x-axis in XZ plane
    cv::Vec3d b = cv::normalize(cv::Vec3d(1.0, 0.0, 1.0));
    double angle = ST::math::AngularSeparationDeg(a, b);
    EXPECT_NEAR(angle, 45.0, 1e-5);
}

TEST(AngularSepTest, DotProductClampPreventsNaN) {
    // Numerically slightly out-of-range dot should still return a valid angle
    // (simulate floating-point rounding: pass un-normalised but ~unit vectors)
    cv::Vec3d a(1.0 + 1e-15, 0.0, 0.0); // just barely > 1 norm
    cv::Vec3d b(1.0, 0.0, 0.0);
    double angle = ST::math::AngularSeparationDeg(a, b);
    EXPECT_FALSE(std::isnan(angle));
    EXPECT_NEAR(angle, 0.0, 1.0); // should be close to 0
}

TEST(AngularSepTest, ResultAlwaysInValidRange) {
    // Angle must be in [0, 180]
    const std::vector<std::pair<cv::Vec3d, cv::Vec3d>> pairs = {
        {{1,0,0},   {0,1,0}},
        {{1,1,0},   {-1,-1,0}},
        {{0,0,1},   {0,0,-1}},
        {{1,2,3},   {-3,-2,-1}},
    };
    for (auto& [a, b] : pairs) {
        cv::Vec3d na = cv::normalize(a);
        cv::Vec3d nb = cv::normalize(b);
        double angle = ST::math::AngularSeparationDeg(na, nb);
        EXPECT_GE(angle, 0.0);
        EXPECT_LE(angle, 180.0);
    }
}

TEST_F(PixelToBodyRayTest, ExtremeOffAxisCoordinatesNormalized) {
    // Large negative and positive coordinates (e.g. out of frame / stray light)
    cv::Vec3d r1 = ST::math::pixel_to_body_ray(-1000.0, -2000.0, fx, fy, cx, cy);
    EXPECT_NEAR(cv::norm(r1), 1.0, kEps);
    EXPECT_LT(r1[0], 0.0);
    EXPECT_LT(r1[1], 0.0);
    EXPECT_GT(r1[2], 0.0);

    cv::Vec3d r2 = ST::math::pixel_to_body_ray(5000.0, 5000.0, fx, fy, cx, cy);
    EXPECT_NEAR(cv::norm(r2), 1.0, kEps);
    EXPECT_GT(r2[0], 0.0);
    EXPECT_GT(r2[1], 0.0);
    EXPECT_GT(r2[2], 0.0);
}

TEST(AngularSepTest, TinySeparationAccurate) {
    // 0.001 degree angle around Y axis
    double angle_rad = 0.001 * M_PI / 180.0;
    cv::Vec3d a(0.0, 0.0, 1.0);
    cv::Vec3d b(std::sin(angle_rad), 0.0, std::cos(angle_rad));

    double angle_deg = ST::math::AngularSeparationDeg(a, b);
    EXPECT_NEAR(angle_deg, 0.001, 1e-6);
}

