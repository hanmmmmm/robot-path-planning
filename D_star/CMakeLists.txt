cmake_minimum_required(VERSION 2.10)
project(D_STAR)


find_package(OpenCV REQUIRED)
# find_package(Boost 1.60 COMPONENTS)

include_directories( ${OpenCV_INCLUDE_DIRS} )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

add_library(utillib utils/img_io.cpp)
add_library(d_star d_star.cpp)

add_executable(d_star_path main.cpp )


target_link_libraries(d_star_path d_star utillib ${OpenCV_LIBS} )










