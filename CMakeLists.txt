cmake_minimum_required (VERSION 3.12.3)
project (OpenCV-Viz)

add_compile_options(-Wall -Wextra -O2 -std=c++11)

add_executable(demo main.cpp utils.cpp)
target_link_libraries(demo -lopencv_core -lopencv_calib3d -lopencv_viz)