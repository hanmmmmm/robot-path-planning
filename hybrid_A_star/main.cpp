#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <stdexcept>

#include "utils/img_io.h"
#include "astar.h"


void find_path(int startnode[], int goalnode[], float start_angle, float goal_angle, cv::Mat map)
{
    astar path_finder(startnode, goalnode,start_angle, goal_angle, map);
    std::cout << "Done init" << std::endl;

    path_finder.search();

    path_finder.get_path();

    path_finder.print_path();
}



int main()
{

    std::string img_path = "map3.png";

    cv::Mat img = setup_map(img_path).clone();

    //  case 1 

    int startnode[2] = {30, 80};  // start position, {x ,y}
    int goalnode[2] = {440, 480 }; // goal  position, {x ,y}
    float start_angle;
    float goal_angle;

    goalnode[0] = 350;  // start position, x
    goalnode[1] = 60;  // start position, x
    start_angle = 0;
    goal_angle  = M_PI/2;
    find_path( startnode , goalnode, start_angle, goal_angle, img);

    goalnode[0] = 200;  // start position, x
    goalnode[1] = 200;  // start position, x
    start_angle = 0;
    goal_angle  = 0;
    find_path( startnode , goalnode, start_angle, goal_angle, img);

    goalnode[0] = 200;  // start position, x
    goalnode[1] = 300;  // start position, x
    start_angle = 0;
    goal_angle  = M_PI/2;
    find_path( startnode , goalnode, start_angle, goal_angle, img);

    goalnode[0] = 190;  // start position, x
    goalnode[1] = 480;  // start position, x
    start_angle = 0;
    goal_angle  = M_PI/2 ;
    find_path( startnode , goalnode, start_angle, goal_angle, img);

    goalnode[0] = 470;  // start position, x
    goalnode[1] = 350;  // start position, x
    start_angle = 0;   
    goal_angle  = M_PI*0.5;
    find_path( startnode , goalnode, start_angle, goal_angle, img);

}
