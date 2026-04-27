add_executable(startracker 
main.cpp
reader.cpp
processor.cpp
config.cpp
logger.cpp
)

target_include_directories(startracker PUBLIC 
    "${PROJECT_SOURCE_DIR}/include"
)

target_compile_options(startracker PRIVATE 
    $<$<CONFIG:Debug>:-g -O0>
)

target_link_libraries(startracker 
${OpenCV_LIBS}
pthread
)

add_compile_options(-Wall -Wextra -Wpedantic)