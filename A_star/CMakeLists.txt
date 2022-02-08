cmake_minimum_required(VERSION 2.10)
project(ASTAR)


find_package(OpenCV REQUIRED)
# find_package(Boost 1.60 COMPONENTS)

include_directories( ${OpenCV_INCLUDE_DIRS} )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

add_library(utillib utils/img_io.cpp)
add_library(astarlib astar.cpp)

add_executable(astar_path main.cpp )


target_link_libraries(astar_path astarlib utillib ${OpenCV_LIBS} )










