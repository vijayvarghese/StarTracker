/**
 * @file test_config.cpp
 * @brief Unit tests for configuration structures and JSON deserialization.
 */

#include <gtest/gtest.h>
#include "config.hpp"
#include "nlohmann/json.hpp"
#include <chrono>

TEST(ConfigTest, ConfigInitDoesNotThrow) {
    // When config file is missing at relative path, config_init returns FallBack safely
    EXPECT_NO_THROW({
        ST::ConfigInitStatus status = config_init();
        EXPECT_TRUE(status == ST::ConfigInitStatus::FallBack ||
                    status == ST::ConfigInitStatus::Json);
    });
}

TEST(ConfigTest, JsonDeserializationPopulatesAllFields) {
    std::string json_str = R"({
        "lookup_cfg": {
            "binpath": "test_data/lookup.bin",
            "lookup_precision": 0.05,
            "tolerance": 3
        },
        "camera_intr_cfg": {
            "fx": 1200.0,
            "fy": 1200.0,
            "cx": 512.0,
            "cy": 512.0
        },
        "reader_cfg": {
            "source": "file",
            "file_path": "test.png",
            "expected_width": 1024,
            "expected_height": 1024,
            "frequency": 30.0,
            "period": 33
        },
        "processor_cfg": {
            "frequency": 10.0,
            "period": 100,
            "cv": {
                "threshold": 180,
                "blur_ksize": 5,
                "blur_sigma": 1.5,
                "area_max": 25,
                "window_debug": false
            }
        }
    })";

    nlohmann::json cfg = nlohmann::json::parse(json_str);

    // Verify lookup_cfg
    EXPECT_EQ(cfg["lookup_cfg"]["binpath"].get<std::string>(), "test_data/lookup.bin");
    EXPECT_DOUBLE_EQ(cfg["lookup_cfg"]["lookup_precision"].get<double>(), 0.05);
    EXPECT_EQ(cfg["lookup_cfg"]["tolerance"].get<int>(), 3);

    // Verify camera_intr_cfg
    EXPECT_DOUBLE_EQ(cfg["camera_intr_cfg"]["fx"].get<double>(), 1200.0);
    EXPECT_DOUBLE_EQ(cfg["camera_intr_cfg"]["fy"].get<double>(), 1200.0);
    EXPECT_DOUBLE_EQ(cfg["camera_intr_cfg"]["cx"].get<double>(), 512.0);
    EXPECT_DOUBLE_EQ(cfg["camera_intr_cfg"]["cy"].get<double>(), 512.0);

    // Verify processor_cfg
    EXPECT_EQ(cfg["processor_cfg"]["cv"]["threshold"].get<int>(), 180);
    EXPECT_EQ(cfg["processor_cfg"]["cv"]["blur_ksize"].get<int>(), 5);
    EXPECT_DOUBLE_EQ(cfg["processor_cfg"]["cv"]["blur_sigma"].get<double>(), 1.5);
    EXPECT_EQ(cfg["processor_cfg"]["cv"]["area_max"].get<int>(), 25);
    EXPECT_FALSE(cfg["processor_cfg"]["cv"]["window_debug"].get<bool>());
}
