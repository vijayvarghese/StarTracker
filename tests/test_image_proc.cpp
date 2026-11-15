/**
 * @file test_image_proc.cpp
 * @brief Unit tests for ST::image (preprocessing and centroid extraction).
 */

#include <gtest/gtest.h>
#include "startracker/image/img_proc.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

// ─── PreprocessFrame Tests ───────────────────────────────────────────────────

TEST(ImageProcTest, EmptyFrameReturnsEmpty) {
    cv::Mat empty_frame;
    cv::Mat result = ST::image::preprocess_frame(empty_frame, 0.0, 3, 1.0, 128);
    EXPECT_TRUE(result.empty());
}

TEST(ImageProcTest, BgrFramePreprocessedSuccessfully) {
    // 100x100 black image with a bright 5x5 square in the center
    cv::Mat frame = cv::Mat::zeros(100, 100, CV_8UC3);
    cv::rectangle(frame, cv::Rect(48, 48, 5, 5), cv::Scalar(255, 255, 255), cv::FILLED);

    cv::Mat bw = ST::image::preprocess_frame(frame, 0.0, 3, 0.5, 100);

    EXPECT_FALSE(bw.empty());
    EXPECT_EQ(bw.channels(), 1);
    EXPECT_EQ(bw.size(), frame.size());
    EXPECT_GT(cv::countNonZero(bw), 0);
    EXPECT_EQ(bw.at<uint8_t>(50, 50), 255);
    EXPECT_EQ(bw.at<uint8_t>(0, 0), 0);
}

TEST(ImageProcTest, GrayscaleFrameHandledDirectly) {
    cv::Mat gray = cv::Mat::zeros(50, 50, CV_8UC1);
    gray.at<uint8_t>(25, 25) = 250;

    cv::Mat bw = ST::image::preprocess_frame(gray, 0.0, 3, 0.5, 100);

    EXPECT_FALSE(bw.empty());
    EXPECT_EQ(bw.channels(), 1);
    EXPECT_GT(cv::countNonZero(bw), 0);
}

TEST(ImageProcTest, ThresholdCutoffApplied) {
    cv::Mat frame = cv::Mat::zeros(10, 10, CV_8UC1);
    frame.at<uint8_t>(2, 2) = 150; // Above 100
    frame.at<uint8_t>(7, 7) = 50;  // Below 100

    cv::Mat bw = ST::image::preprocess_frame(frame, 0.0, 1, 0.0, 100);

    EXPECT_EQ(bw.at<uint8_t>(2, 2), 255);
    EXPECT_EQ(bw.at<uint8_t>(7, 7), 0);
}

TEST(ImageProcTest, EvenKernelSizeAdjustedWithoutThrowing) {
    cv::Mat frame = cv::Mat::zeros(50, 50, CV_8UC3);
    frame.at<cv::Vec3b>(25, 25) = cv::Vec3b(255, 255, 255);

    // blur_ksize = 4 is even; should be adjusted internally without assertion failure
    EXPECT_NO_THROW({
        cv::Mat bw = ST::image::preprocess_frame(frame, 0.0, 4, 1.0, 50);
        EXPECT_FALSE(bw.empty());
    });
}

// ─── Centroid Extraction Tests ───────────────────────────────────────────────

TEST(CentroidTest, EmptyBinaryReturnsEmptyList) {
    cv::Mat empty_mat;
    auto centroids = ST::image::get_centroids(empty_mat, 0.0, 50);
    EXPECT_TRUE(centroids.empty());
}

TEST(CentroidTest, AllBlackImageReturnsEmptyList) {
    cv::Mat black = cv::Mat::zeros(100, 100, CV_8UC1);
    auto centroids = ST::image::get_centroids(black, 0.0, 50);
    EXPECT_TRUE(centroids.empty());
}

TEST(CentroidTest, SingleStarDetectedAccurately) {
    cv::Mat img = cv::Mat::zeros(100, 100, CV_8UC1);
    // Draw a 3x3 square centered around (40, 60): x in [39, 41], y in [59, 61]
    cv::rectangle(img, cv::Rect(39, 59, 3, 3), cv::Scalar(255), cv::FILLED);

    auto centroids = ST::image::get_centroids(img, 0.0, 50);

    ASSERT_EQ(centroids.size(), 1u);
    EXPECT_NEAR(centroids[0].x, 40.0, 0.1);
    EXPECT_NEAR(centroids[0].y, 60.0, 0.1);
}

TEST(CentroidTest, MultipleSeparatedStarsDetected) {
    cv::Mat img = cv::Mat::zeros(200, 200, CV_8UC1);
    // 3 distinct stars
    cv::rectangle(img, cv::Rect(29, 29, 3, 3), cv::Scalar(255), cv::FILLED); // ~ (30, 30)
    cv::rectangle(img, cv::Rect(99, 49, 3, 3), cv::Scalar(255), cv::FILLED); // ~ (100, 50)
    cv::rectangle(img, cv::Rect(149, 149, 3, 3), cv::Scalar(255), cv::FILLED); // ~ (150, 150)

    auto centroids = ST::image::get_centroids(img, 0.0, 50);

    EXPECT_EQ(centroids.size(), 3u);
}

TEST(CentroidTest, AreaMaxFiltersLargeBlobs) {
    cv::Mat img = cv::Mat::zeros(100, 100, CV_8UC1);
    // 1 small star: 2x2 = 4 pixels (below area_max=10)
    cv::rectangle(img, cv::Rect(10, 10, 2, 2), cv::Scalar(255), cv::FILLED);
    // 1 large blob: 10x10 = 100 pixels (exceeds area_max=10)
    cv::rectangle(img, cv::Rect(50, 50, 10, 10), cv::Scalar(255), cv::FILLED);

    auto centroids = ST::image::get_centroids(img, 0.0, 10);

    ASSERT_EQ(centroids.size(), 1u);
    EXPECT_NEAR(centroids[0].x, 10.5, 0.5);
    EXPECT_NEAR(centroids[0].y, 10.5, 0.5);
}
