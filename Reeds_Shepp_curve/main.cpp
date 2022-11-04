#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <stdexcept>

#include "utils/img_io.h"
#include "reedsshepp.h"



int main()
{
    

    std::string img_path = "map3.png";

    cv::Mat img = setup_map(img_path).clone();

    ReedsSheppClass path_finder;

    int startnode[2] = {270, 270};  // start position, {x ,y}
    int goalnode[2] = {440, 480 }; // goal  position, {x ,y}
    float start_angle;
    float goal_angle;

    goalnode[0] = 350;  // start position, x
    goalnode[1] = 360;// 60;  // start position, x
    
    start_angle = 0 ;  // 2.9;
    goal_angle  = 0 ;  //M_PI/2;
    
    float pose2_pose_direction_step = 20 * M_PI / 180.0;
    float pose2_pose_direction = 0;
    int distance = 200;


    goalnode[0] = int(270 + distance * cos(pose2_pose_direction));  // start position, x
    goalnode[1] = int(270 + distance * sin(pose2_pose_direction));
    path_finder.setup( startnode , goalnode, start_angle, goal_angle, img );
    path_finder.search(true);

    while( pose2_pose_direction < 2*M_PI )
    {
        float pose2_x = 270 + distance * cos(pose2_pose_direction);
        float pose2_y = 270 + distance * sin(pose2_pose_direction);
        goal_angle  = pose2_pose_direction*2;
        goalnode[0] = int(pose2_x);  // start position, x
        goalnode[1] = int(pose2_y);
        path_finder.setup( startnode , goalnode, start_angle, goal_angle, img );
        path_finder.search(true);
        // find_path( startnode , goalnode, start_angle, goal_angle, img);

        pose2_pose_direction += pose2_pose_direction_step;

    }

    // pose2_pose_direction = 0;
    // distance = 100;

    // while( pose2_pose_direction < 2*M_PI )
    // {
    //     float pose2_x = 270 + distance * cos(pose2_pose_direction);
    //     float pose2_y = 270 + distance * sin(pose2_pose_direction);
    //     goal_angle  = pose2_pose_direction*2;
    //     goalnode[0] = int(pose2_x);  // start position, x
    //     goalnode[1] = int(pose2_y);
    //     path_finder.setup( startnode , goalnode, start_angle, goal_angle, img );
    //     path_finder.search(false);
    //     // find_path( startnode , goalnode, start_angle, goal_angle, img);

    //     pose2_pose_direction += pose2_pose_direction_step;

    // }

    // goalnode[0] = int(270 + distance * cos(pose2_pose_direction));  // start position, x
    // goalnode[1] = int(270 + distance * sin(pose2_pose_direction));
    // path_finder.setup( startnode , goalnode, start_angle, goal_angle, img );
    // path_finder.search(true);

}
