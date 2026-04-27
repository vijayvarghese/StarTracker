cmake_minimum_required(VERSION 3.10)
project(StarTracker VERSION 1.0 LANGUAGES C CXX)

find_package(OpenCV REQUIRED)


add_subdirectory(src)
add_subdirectory(scripts)
add_subdirectory(external/nlohmann_json)
add_subdirectory(external/ftxui)




