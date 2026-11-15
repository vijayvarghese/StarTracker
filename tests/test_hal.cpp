/**
 * @file test_hal.cpp
 * @brief Unit tests for Camera HAL classes (UnityMockFile).
 */

#include <gtest/gtest.h>
#include "startracker/hal/UnityMockFile.hpp"
#include <filesystem>
#include <opencv2/imgcodecs.hpp>

namespace fs = std::filesystem;

TEST(HalTest, UnityMockFileNonExistentPathReturnsEmpty) {
    ST::UnityMockFile cam("/path/does/not/exist/image_99999.png");
    EXPECT_TRUE(cam.isOpen());

    cv::Mat frame = cam.grabFrame();
    EXPECT_TRUE(frame.empty());

    cam.close();
}

TEST(HalTest, UnityMockFileReadsValidImage) {
    std::string tmp_path = (fs::temp_directory_path() / "st_test_hal_img.png").string();

    // Create a 64x64 test image
    cv::Mat original = cv::Mat::zeros(64, 64, CV_8UC3);
    original.at<cv::Vec3b>(32, 32) = cv::Vec3b(255, 255, 255);
    ASSERT_TRUE(cv::imwrite(tmp_path, original));

    ST::UnityMockFile cam(tmp_path);
    EXPECT_TRUE(cam.isOpen());

    cv::Mat loaded = cam.grabFrame();
    EXPECT_FALSE(loaded.empty());
    EXPECT_EQ(loaded.cols, 64);
    EXPECT_EQ(loaded.rows, 64);

    cam.close();
    fs::remove(tmp_path);
}
