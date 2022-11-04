cmake_minimum_required(VERSION 2.10)
project(RS_curve)


find_package(OpenCV REQUIRED)
# find_package(Boost 1.60 COMPONENTS)

include_directories( ${OpenCV_INCLUDE_DIRS} )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

add_library(utillib utils/img_io.cpp)
# add_library(dubins dubins.cpp)

include_directories( reedsshepp_func )

add_executable(reedsshepp_path 
main.cpp 
reedsshepp.cpp
reedsshepp_func/reedsshepp_print.cpp 
reedsshepp_func/reedsshepp_compute.cpp 
reedsshepp_func/reedsshepp_draw.cpp
)


# target_link_libraries(dubins_path dubins utillib ${OpenCV_LIBS} )



target_link_libraries(reedsshepp_path utillib ${OpenCV_LIBS} )







