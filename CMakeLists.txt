# CMakeのバージョンを設定
cmake_minimum_required(VERSION 3.22)
# プロジェクト名と使用する言語を設定
project(object_detection CXX)
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)
add_compile_options(-g -O0)
add_executable(a.out main.cpp)
target_link_libraries(a.out
    PRIVATE
        realsense2
        ${OpenCV_LIBS}
)