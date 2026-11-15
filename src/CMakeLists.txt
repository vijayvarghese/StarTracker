add_executable(startracker 
main.cpp
reader.cpp
processor.cpp
config.cpp
core/logger.cpp
image/img_proc.cpp
math/geometry.cpp
identification/n-star/n-star.cpp
hal/UnityMockFile.cpp
hal/UnityMockTCP.cpp
util/dbg/debug_utils.cpp
catalog/loader.cpp
attitude/quest.cpp
)

target_compile_features(startracker PRIVATE cxx_std_20)

target_include_directories(startracker PUBLIC 
    "${PROJECT_SOURCE_DIR}/include/"
)

target_compile_options(startracker PRIVATE 
    $<$<CONFIG:Debug>:-g>
)

target_link_libraries(startracker 
PRIVATE
${OpenCV_LIBS}
m
pthread
nlohmann_json::nlohmann_json
Eigen3::Eigen
)

if(TUI_BUILD)
    target_sources(startracker PRIVATE
    ui/tui.cpp
    )

    target_link_libraries(startracker
        PRIVATE
        ftxui::screen
        ftxui::dom
        ftxui::component
    )

    target_compile_definitions(
        startracker
        PRIVATE
        ST_TUI_BUILD
    )
endif()
if(UNITY_TCP_CAM)
    target_compile_definitions(
        startracker
        PRIVATE
        UNITY_TCP_CAM
    )
endif()
add_compile_options(-Wall -Wextra -Wpedantic)
