#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <stdexcept>

#include "utils/img_io.h"
#include "prm.h"


void find_path(int startnode[], int goalnode[], cv::Mat map)
{
    Prm path_finder(startnode, goalnode, map);
    std::cout << "Done init" << std::endl;

    path_finder.search();

    path_finder.get_path();

}



int main()
{

    std::string img_path = "map3.png";

    cv::Mat img = setup_map(img_path).clone();

    int startnode[2] = {50, 100};  // start position, {x ,y}
    int goalnode[2] = {470, 350 }; // goal  position, {x ,y}

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
