cmake_minimum_required(VERSION 3.10)
project(StarTracker)

find_package(OpenCV REQUIRED)

add_executable(startracker 
src/main.cpp
src/reader.cpp
src/processor.cpp
src/config.cpp
)

target_link_libraries(startracker 
${OpenCV_LIBS}
pthread
)

add_compile_options(-Wall -Wextra -Wpedantic)
