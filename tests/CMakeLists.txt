cmake_minimum_required(VERSION 3.14)

# ─── Fetch GoogleTest ──────────────────────────────────────────────────────────
include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG        v1.14.0
)
# Prevent overriding the parent project's compiler/linker settings on Windows
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()
include(GoogleTest)

# ─── Common library compiled from production sources ─────────────────────────
# We build a static library containing the testable modules so each test
# executable doesn't have to list individual sources.
add_library(st_testable STATIC
    ${PROJECT_SOURCE_DIR}/src/math/geometry.cpp
    ${PROJECT_SOURCE_DIR}/src/attitude/quest.cpp
    ${PROJECT_SOURCE_DIR}/src/identification/n-star/n-star.cpp
    ${PROJECT_SOURCE_DIR}/src/catalog/loader.cpp
    ${PROJECT_SOURCE_DIR}/src/core/logger.cpp
    ${PROJECT_SOURCE_DIR}/src/util/dbg/debug_utils.cpp
    ${PROJECT_SOURCE_DIR}/src/config.cpp
    ${PROJECT_SOURCE_DIR}/src/image/img_proc.cpp
    ${PROJECT_SOURCE_DIR}/src/hal/UnityMockFile.cpp
    # Provides global atomics (log_level, ST::dbg::*) that live in main.cpp
    # in the production build.
    test_globals.cpp
)

target_include_directories(st_testable PUBLIC
    ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(st_testable PUBLIC
    ${OpenCV_LIBS}
    Eigen3::Eigen
    nlohmann_json::nlohmann_json
    m
)

target_compile_features(st_testable PRIVATE cxx_std_20)

# ─── Helper macro ─────────────────────────────────────────────────────────────
# Usage: st_add_test(<name> <source1> [<source2> ...])
macro(st_add_test name)
    add_executable(${name} ${ARGN})
    target_link_libraries(${name} PRIVATE st_testable GTest::gtest_main)
    target_compile_features(${name} PRIVATE cxx_std_20)
    # Register with CTest — runs the binary, GTest handles discovery internally.
    add_test(NAME ${name} COMMAND ${name})
    # Propagate GTest output back to CTest on failure.
    set_tests_properties(${name} PROPERTIES
        ENVIRONMENT "GTEST_COLOR=1"
    )
endmacro()

# ─── Test suites ──────────────────────────────────────────────────────────────
st_add_test(test_math             test_math.cpp)
st_add_test(test_attitude         test_attitude.cpp)
st_add_test(test_identification   test_identification.cpp)
st_add_test(test_catalog_loader   test_catalog_loader.cpp)
st_add_test(test_image_proc       test_image_proc.cpp)
st_add_test(test_config           test_config.cpp)
st_add_test(test_hal              test_hal.cpp)
