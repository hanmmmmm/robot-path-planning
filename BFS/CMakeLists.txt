cmake_minimum_required(VERSION 2.10)
project(BFS)


find_package(OpenCV REQUIRED)
find_package(Boost 1.60 COMPONENTS)

include_directories( ${OpenCV_INCLUDE_DIRS} )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

add_library(utillib utils/img_io.cpp)
add_library(bfslib bfs.cpp)

add_executable(bfs_path main.cpp )


target_link_libraries(bfs_path bfslib utillib ${OpenCV_LIBS} )










