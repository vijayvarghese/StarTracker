/**
 * @file test_globals.cpp
 * @brief Provides definitions for global singletons that live in main.cpp in
 *        the production build but must be present for test linking.
 *
 * In the production binary these are defined in src/main.cpp.
 * The test library (st_testable) does NOT include main.cpp, so we define
 * them here instead.
 */

#include "startracker/core/logger.hpp"
#include "startracker/util/dbg/debug_utils.hpp"
#include <atomic>

// ─── Logger ──────────────────────────────────────────────────────────────────
namespace ST::core::log {
std::atomic<int> log_level = 0; // 0 = minimal output during tests
}

// ─── Debug flags (ST::dbg) ───────────────────────────────────────────────────
namespace ST::dbg {
std::atomic<bool> centroid = false;
std::atomic<bool> ray = false;
std::atomic<bool> ang_sep = false;
std::atomic<bool> img = false;
std::atomic<bool> ang_profile = false;
std::atomic<bool> quest = false;
} // namespace ST::dbg
