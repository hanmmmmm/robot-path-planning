cmake_minimum_required(VERSION 2.10)
project(RRT)


find_package(OpenCV REQUIRED)
# find_package(Boost 1.60 COMPONENTS)

include_directories( ${OpenCV_INCLUDE_DIRS} )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

add_library(utillib utils/img_io.cpp)
add_library(rrtlib rrt_star.cpp)

add_executable(rrt_star_path 
main.cpp 
member_func/member_print.cpp 
# member_func/member_compute.cpp 
member_func/member_draw.cpp
)


target_link_libraries(rrt_star_path rrtlib utillib ${OpenCV_LIBS} )










