cmake_minimum_required(VERSION 3.10)
set(CMAKE_C_COMPILER clang)
set(CMAKE_CXX_COMPILER clang++)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
option(TUI_BUILD "Build with FTXUI interface" OFF)

project(StarTracker VERSION 1.0 LANGUAGES C CXX)
   
find_package(OpenCV REQUIRED)


add_subdirectory(src)
add_subdirectory(scripts)
add_subdirectory(external/nlohmann_json)
if(TUI_BUILD)
add_subdirectory(external/ftxui)
endif()



