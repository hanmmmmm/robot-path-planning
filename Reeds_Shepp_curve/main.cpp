#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <stdexcept>
#include <array>

#include "reedsshepp.h"
#include "reedshepp_path.h"
#include "visulizar_opencv.h" 


int main()
{
    ReedsSheppClass path_finder;

    ClassRScurveOpencv vis; 

    // int startnode[2] = {270, 270};  // start position, {x ,y}
    // int goalnode[2] = {440, 480 }; // goal  position, {x ,y}
    // float start_angle;
    // float goal_angle;

    // goalnode[0] = 350;  // start position, x
    // goalnode[1] = 360;// 60;  // start position, x
    
    // start_angle = 0 ;  // 2.9;
    // goal_angle  = 0 ;  //M_PI/2;
    
    std::array<double,3> start_pose = {2.2, 0.3, 0.0};
    std::array<double,3> goal_pose  = { 6.5, 0.3, 0.0};

    bool manual = false;
    manual = true; // comment out this line to use animation mode. 

    
    float pose2_pose_direction_step ;
    if( manual )   pose2_pose_direction_step  = 10 * M_PI / 180.0;
    else pose2_pose_direction_step  = 0.5 * M_PI / 180.0;

    double goal_direction = 0;  // 
    double distance = 3.7;  // meter 

    int loop_count = 0;
    while (goal_direction < 2*M_PI )
    {
        goal_pose[0] = start_pose[0] + distance * cos(goal_direction);
        goal_pose[1] = start_pose[1] + distance * sin(goal_direction);
        goal_pose[2] = goal_direction * 2.0 ;
        goal_direction += pose2_pose_direction_step;

        path_finder.setup( start_pose, goal_pose);
        path_finder.search();

        vis.init();

        vis.draw_vector_on_map(start_pose);
        vis.draw_vector_on_map( goal_pose);

        
        vis.draw_path_by_cvline( path_finder.all_possible_paths_.LpSpLp );
        vis.draw_path_by_cvline( path_finder.all_possible_paths_.LpSpRp );
        vis.draw_path_by_cvline( path_finder.all_possible_paths_.LmSmLm );
        vis.draw_path_by_cvline( path_finder.all_possible_paths_.LmSmRm );

        vis.draw_path_by_cvline( path_finder.all_possible_paths_.RpSpLp );
        vis.draw_path_by_cvline( path_finder.all_possible_paths_.RpSpRp );
        vis.draw_path_by_cvline( path_finder.all_possible_paths_.RmSmRm );
        vis.draw_path_by_cvline( path_finder.all_possible_paths_.RmSmLm );

        vis.draw_path_by_cvline( path_finder.all_possible_paths_.LpRmL );
        vis.draw_path_by_cvline( path_finder.all_possible_paths_.LmRpLm );
        vis.draw_path_by_cvline( path_finder.all_possible_paths_.RpLmRp );
        vis.draw_path_by_cvline( path_finder.all_possible_paths_.RmLpRm );

        vis.draw_path_by_cvline( path_finder.all_possible_paths_.LpRmLm );
        vis.draw_path_by_cvline( path_finder.all_possible_paths_.LmRpLp );
        vis.draw_path_by_cvline( path_finder.all_possible_paths_.RpLmRm );
        vis.draw_path_by_cvline( path_finder.all_possible_paths_.RmLpRp );


        vis.show(manual);
        
        

        // for ( auto each : path_finder.all_possible_paths_ ){

        // }
    }
    
}
