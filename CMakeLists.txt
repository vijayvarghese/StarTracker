cmake_minimum_required(VERSION 3.10)
project(StarTracker C CXX)

find_package(OpenCV REQUIRED)

add_subdirectory(scripts)

add_executable(startracker 
src/main.cpp
src/reader.cpp
src/processor.cpp
src/config.cpp
src/logger.cpp
)

target_compile_options(startracker PRIVATE 
    $<$<CONFIG:Debug>:-g -O0>
)

target_link_libraries(startracker 
${OpenCV_LIBS}
pthread
)

add_compile_options(-Wall -Wextra -Wpedantic)
