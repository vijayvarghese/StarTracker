add_executable(startracker 
main.cpp
reader.cpp
processor.cpp
config.cpp
logger.cpp
tui.cpp
img_proc.cpp
geometry.cpp
core.cpp
hal/UnityMock.cpp
)

target_compile_features(startracker PRIVATE cxx_std_20)

target_include_directories(startracker PUBLIC 
    "${PROJECT_SOURCE_DIR}/include"
    "${PROJECT_SOURCE_DIR}/include/startracker"
    "${PROJECT_SOURCE_DIR}/include/startracker/hal"
)

target_compile_options(startracker PRIVATE 
    $<$<CONFIG:Debug>:-g -O0>
)

target_link_libraries(startracker 
${OpenCV_LIBS}
pthread
nlohmann_json::nlohmann_json
ftxui::screen
ftxui::dom
ftxui::component
)

add_compile_options(-Wall -Wextra -Wpedantic)