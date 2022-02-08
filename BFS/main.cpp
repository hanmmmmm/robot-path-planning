#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
// #include <stdexcept>

#include "utils/img_io.h"
#include "bfs.h"


void find_path(int startnode[], int goalnode[], cv::Mat map)
{
    bfs path_finder(startnode, goalnode, map);

    path_finder.search();

    path_finder.get_path_no_return();

    path_finder.print_path();
}



int main()
{

    std::string img_path = "../map3.png";

    cv::Mat img = setup_map(img_path).clone();


    int startnode[2] = {3, 8};  // start position, {x ,y}
    int goalnode[2] = {44, 48}; // goal  position, {x ,y}


    goalnode[0] = 35;  // start position, x
    goalnode[1] = 6;  // start position, x
    find_path( startnode , goalnode, img);

    goalnode[0] = 20;  // start position, x
    goalnode[1] = 20;  // start position, x
    find_path( startnode , goalnode, img);

    goalnode[0] = 20;  // start position, x
    goalnode[1] = 30;  // start position, x
    find_path( startnode , goalnode, img);

    goalnode[0] = 19;  // start position, x
    goalnode[1] = 48;  // start position, x
    find_path( startnode , goalnode, img);

    goalnode[0] = 47;  // start position, x
    goalnode[1] = 35;  // start position, x
    find_path( startnode , goalnode, img);

}