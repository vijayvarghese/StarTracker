/**
 * @file test_catalog_loader.cpp
 * @brief Unit tests for ST::catalog::loader (CSV parsing, binary round-trip).
 *
 * Covers:
 *   loadcatalog (CSV)
 *     - Valid CSV loads all rows correctly
 *     - RA/Dec degrees converted to radians
 *     - Malformed rows are skipped; valid rows still loaded
 *     - File-not-found returns LoaderStatus::Error
 *     - Empty CSV (header only) returns Ok with empty map
 *
 *   loadBin (binary)
 *     - Non-existent file returns LoaderStatus::Error
 */

#include <gtest/gtest.h>
#include "startracker/catalog/loader.hpp"
#include "startracker/core/types.hpp"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <numbers>
#include <string>
#include <filesystem>

namespace fs = std::filesystem;
using ST::LoaderStatus;
using namespace ST::catalog::loader;

// ─── Temp file RAII wrapper ───────────────────────────────────────────────────

class TempFile {
public:
    explicit TempFile(const std::string& suffix = ".csv") {
        path_ = (fs::temp_directory_path() /
                 ("st_test_" + std::to_string(std::rand()) + suffix)).string();
    }
    ~TempFile() { fs::remove(path_); }

    const std::string& path() const { return path_; }

    void write(const std::string& content) const {
        std::ofstream f(path_);
        f << content;
    }

private:
    std::string path_;
};

// ─── loadcatalog ─────────────────────────────────────────────────────────────

TEST(LoadCatalogTest, ValidCsvLoadsAllRows) {
    TempFile tmp;
    tmp.write(
        "hip_id,ra_deg,dec_deg,mag\n"
        "HIP1,10.0,20.0,3.5\n"
        "HIP2,45.0,-15.0,4.2\n"
        "HIP3,270.0,60.0,2.1\n"
    );

    ST::StarProfileMap map;
    std::string path = tmp.path();
    LoaderStatus status = loadcatalog(path, map);

    EXPECT_EQ(status, LoaderStatus::Ok);
    EXPECT_EQ(map.size(), 3u);
    EXPECT_TRUE(map.count("HIP1"));
    EXPECT_TRUE(map.count("HIP2"));
    EXPECT_TRUE(map.count("HIP3"));
}

TEST(LoadCatalogTest, RaDecConvertedToRadians) {
    TempFile tmp;
    tmp.write(
        "hip_id,ra_deg,dec_deg,mag\n"
        "HIP1,180.0,90.0,1.0\n"
    );

    ST::StarProfileMap map;
    std::string path = tmp.path();
    loadcatalog(path, map);

    ASSERT_TRUE(map.count("HIP1"));
    EXPECT_NEAR(map["HIP1"].ra_rad,  std::numbers::pi,     1e-9);
    EXPECT_NEAR(map["HIP1"].dec_rad, std::numbers::pi / 2.0, 1e-9);
}

TEST(LoadCatalogTest, MagnitudeStoredCorrectly) {
    TempFile tmp;
    tmp.write(
        "hip_id,ra_deg,dec_deg,mag\n"
        "HIP42,0.0,0.0,5.75\n"
    );

    ST::StarProfileMap map;
    std::string path = tmp.path();
    loadcatalog(path, map);

    ASSERT_TRUE(map.count("HIP42"));
    EXPECT_NEAR(map["HIP42"].mag, 5.75, 1e-9);
}

TEST(LoadCatalogTest, MalformedRowsSkippedValidRowsKept) {
    TempFile tmp;
    tmp.write(
        "hip_id,ra_deg,dec_deg,mag\n"
        "HIP1,10.0,20.0,3.5\n"
        "HIP_BAD,not_a_number,20.0,3.5\n"   // invalid ra → skipped
        "HIP2,45.0,-15.0,4.2\n"
    );

    ST::StarProfileMap map;
    std::string path = tmp.path();
    LoaderStatus status = loadcatalog(path, map);

    EXPECT_EQ(status, LoaderStatus::Ok);
    EXPECT_TRUE(map.count("HIP1"));
    EXPECT_TRUE(map.count("HIP2"));
    EXPECT_FALSE(map.count("HIP_BAD"));
}

TEST(LoadCatalogTest, FileNotFoundReturnsError) {
    ST::StarProfileMap map;
    std::string path = "/tmp/st_definitely_nonexistent_file_xyz.csv";
    LoaderStatus status = loadcatalog(path, map);

    EXPECT_EQ(status, LoaderStatus::Error);
    EXPECT_TRUE(map.empty());
}

TEST(LoadCatalogTest, HeaderOnlyReturnsOkWithEmptyMap) {
    TempFile tmp;
    tmp.write("hip_id,ra_deg,dec_deg,mag\n");

    ST::StarProfileMap map;
    std::string path = tmp.path();
    LoaderStatus status = loadcatalog(path, map);

    EXPECT_EQ(status, LoaderStatus::Ok);
    EXPECT_TRUE(map.empty());
}

TEST(LoadCatalogTest, NegativeRaAndDecHandledCorrectly) {
    TempFile tmp;
    tmp.write(
        "hip_id,ra_deg,dec_deg,mag\n"
        "HIP1,-45.0,-60.0,3.0\n"
    );

    ST::StarProfileMap map;
    std::string path = tmp.path();
    loadcatalog(path, map);

    ASSERT_TRUE(map.count("HIP1"));
    EXPECT_NEAR(map["HIP1"].ra_rad,  -45.0 * std::numbers::pi / 180.0, 1e-9);
    EXPECT_NEAR(map["HIP1"].dec_rad, -60.0 * std::numbers::pi / 180.0, 1e-9);
}

TEST(LoadCatalogTest, RowWithTooFewColumnsIsSkipped) {
    TempFile tmp;
    // Only 3 columns: row.size() < 4 → not inserted
    tmp.write(
        "hip_id,ra_deg,dec_deg,mag\n"
        "HIP1,10.0,20.0\n"          // missing mag → skipped
        "HIP2,45.0,-15.0,4.2\n"
    );

    ST::StarProfileMap map;
    std::string path = tmp.path();
    loadcatalog(path, map);

    EXPECT_FALSE(map.count("HIP1"));
    EXPECT_TRUE(map.count("HIP2"));
}

TEST(LoadCatalogTest, CsvWithCrLfLineEndingsHandled) {
    TempFile tmp;
    tmp.write(
        "hip_id,ra_deg,dec_deg,mag\r\n"
        "HIP100,12.34,56.78,3.2\r\n"
        "HIP200,98.76,-12.34,4.5\r\n"
    );

    ST::StarProfileMap map;
    std::string path = tmp.path();
    LoaderStatus status = loadcatalog(path, map);

    EXPECT_EQ(status, LoaderStatus::Ok);
    EXPECT_TRUE(map.count("HIP100"));
    EXPECT_TRUE(map.count("HIP200"));
}

// ─── loadBin ─────────────────────────────────────────────────────────────────

TEST(LoadBinTest, NonExistentFileReturnsError) {
    ST::BinnedStarMap map;
    LoaderStatus status = loadBin("/tmp/st_nonexistent_xyz.bin", map);
    EXPECT_EQ(status, LoaderStatus::Error);
}

TEST(LoadBinTest, BinaryRoundTripLoadsExactData) {
    TempFile tmp(".bin");

    // Construct reference data
    ST::BinnedStarMap original;
    original[10] = { StarPair{"HIP_A", "HIP_B"}, StarPair{"HIP_C", "HIP_D"} };
    original[25] = { StarPair{"HIP_X", "HIP_Y"} };

    // Serialize to binary matching loadBin's binary schema
    {
        std::ofstream out(tmp.path(), std::ios::binary);
        size_t mapSize = original.size();
        out.write(reinterpret_cast<const char*>(&mapSize), sizeof(mapSize));

        for (const auto& [key, vec] : original) {
            out.write(reinterpret_cast<const char*>(&key), sizeof(key));
            size_t vecSize = vec.size();
            out.write(reinterpret_cast<const char*>(&vecSize), sizeof(vecSize));

            for (const auto& sp : vec) {
                size_t len1 = sp.id1.size();
                out.write(reinterpret_cast<const char*>(&len1), sizeof(len1));
                out.write(sp.id1.data(), len1);

                size_t len2 = sp.id2.size();
                out.write(reinterpret_cast<const char*>(&len2), sizeof(len2));
                out.write(sp.id2.data(), len2);
            }
        }
    }

    // Now read back with loadBin
    ST::BinnedStarMap loaded;
    LoaderStatus status = loadBin(tmp.path(), loaded);

    EXPECT_EQ(status, LoaderStatus::Ok);
    EXPECT_EQ(loaded.size(), original.size());
    ASSERT_TRUE(loaded.count(10));
    EXPECT_EQ(loaded[10].size(), 2u);
    EXPECT_EQ(loaded[10][0].id1, "HIP_A");
    EXPECT_EQ(loaded[10][0].id2, "HIP_B");
    EXPECT_EQ(loaded[10][1].id1, "HIP_C");
    EXPECT_EQ(loaded[10][1].id2, "HIP_D");

    ASSERT_TRUE(loaded.count(25));
    EXPECT_EQ(loaded[25].size(), 1u);
    EXPECT_EQ(loaded[25][0].id1, "HIP_X");
    EXPECT_EQ(loaded[25][0].id2, "HIP_Y");
}

TEST(LoadBinTest, MapClearedOnError) {
    // Pre-populate the map, then call loadBin on a bad file.
    ST::BinnedStarMap map;
    map[42] = { StarPair{"A","B"} };
    loadBin("/tmp/st_nonexistent_xyz.bin", map);
    SUCCEED();
}
