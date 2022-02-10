#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <stdexcept>

#include "utils/img_io.h"
#include "rrt.h"


void find_path(std::array<int, 2> &startnode, std::array<int, 2> &goalnode, cv::Mat map)
{
    RRT path_finder(startnode, goalnode, map);
    std::cout << "Done init" << std::endl;

    path_finder.search();

    path_finder.get_path();

}



int main()
{

    std::string img_path = "map3.png";

    cv::Mat img = setup_map(img_path).clone();

    std::array<int, 2>  startnode = {50, 100};  // start position, {x ,y}
    std::array<int, 2>  goalnode = {470, 350 }; // goal  position, {x ,y}

    find_path( startnode , goalnode, img);

    find_path( startnode , goalnode, img);

    find_path( startnode , goalnode, img);

    find_path( startnode , goalnode, img);

    find_path( startnode , goalnode, img);

    find_path( startnode , goalnode, img);

    find_path( startnode , goalnode, img);

    find_path( startnode , goalnode, img);

    find_path( startnode , goalnode, img);

}
