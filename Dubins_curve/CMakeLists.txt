cmake_minimum_required(VERSION 2.10)
project(ASTAR)


find_package(OpenCV REQUIRED)
# find_package(Boost 1.60 COMPONENTS)

include_directories( ${OpenCV_INCLUDE_DIRS} )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

add_library(utillib utils/img_io.cpp)
# add_library(dubins dubins.cpp)

add_executable(dubins_path 
main.cpp 
dubins.cpp
dubins_func/dubins_print.cpp 
dubins_func/dubins_compute.cpp 
dubins_func/dubins_draw.cpp
)


# target_link_libraries(dubins_path dubins utillib ${OpenCV_LIBS} )



target_link_libraries(dubins_path utillib ${OpenCV_LIBS} )







