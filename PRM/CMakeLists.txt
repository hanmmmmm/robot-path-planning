cmake_minimum_required(VERSION 2.10)
project(PRM)


find_package(OpenCV REQUIRED)
# find_package(Boost 1.60 COMPONENTS)

include_directories( ${OpenCV_INCLUDE_DIRS} )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

add_library(utillib utils/img_io.cpp)
add_library(prmlib prm.cpp)

add_executable(prm_path 
main.cpp 
member_func/member_print.cpp 
# member_func/member_compute.cpp 
member_func/member_draw.cpp
)


target_link_libraries(prm_path prmlib utillib ${OpenCV_LIBS} )










