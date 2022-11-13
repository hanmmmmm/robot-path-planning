#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <stdexcept>
#include <array>
#include <random>
#include <chrono>

#include "reedsshepp.h"
#include "reedshepp_path.h"
#include "visulizar_opencv.h" 


int main()
{
    ReedsSheppClass path_finder;

    ClassRScurveOpencv vis; 

    std::random_device dev;

    std::mt19937 rng(dev());
    std::uniform_int_distribution<std::mt19937::result_type> dist6(1,50); 

    std::array<double,3> start_pose = {2.2, 0.3, 0.0};
    std::array<double,3> goal_pose ; // = { 6.5, 0.3, 0.0};

    bool manual = false;
    manual = true; // comment out this line to use animation mode. 

    
    float pose2_pose_direction_step ;
    if( manual )   pose2_pose_direction_step  = 10 * M_PI / 180.0;
    else pose2_pose_direction_step  = 0.5 * M_PI / 180.0;

    double goal_direction = 0;  // 
    double distance = 3.2;  // meter 

    int loop_count = 0;

    while ( loop_count  < 50)
    {
        start_pose[0]= 0.5 + (float(dist6(rng))-25.0)/10.0;
        start_pose[1]= 0.5 + (float(dist6(rng))-25.0)/10.0;
        start_pose[2] = dist6(rng);

        goal_pose[0] = start_pose[0] + (float(dist6(rng))-25.0)/10.0;
        goal_pose[1] = start_pose[1] + (float(dist6(rng))-25.0)/10.0;
        goal_pose[2] = dist6(rng);

        auto rs_start_time =  std::chrono::high_resolution_clock::now();

        path_finder.setup( start_pose, goal_pose);
        path_finder.search();

        auto rs_end_time =  std::chrono::high_resolution_clock::now();

        int duration = std::chrono::duration_cast<std::chrono::microseconds>(rs_end_time - rs_start_time).count();

        std::cout << "Time:  "  << duration << "  us" << std::endl;

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


        vis.draw_path_by_cvline( path_finder.all_possible_paths_.LpRpLp );
        vis.draw_path_by_cvline( path_finder.all_possible_paths_.LpRpLm );
        vis.draw_path_by_cvline( path_finder.all_possible_paths_.LpRmLp );
        vis.draw_path_by_cvline( path_finder.all_possible_paths_.LpRmLm );
        vis.draw_path_by_cvline( path_finder.all_possible_paths_.LmRpLp );
        vis.draw_path_by_cvline( path_finder.all_possible_paths_.LmRpLm );
        vis.draw_path_by_cvline( path_finder.all_possible_paths_.LmRmLp );
        vis.draw_path_by_cvline( path_finder.all_possible_paths_.LmRmLm );

        vis.draw_path_by_cvline( path_finder.all_possible_paths_.RpLpRp );
        vis.draw_path_by_cvline( path_finder.all_possible_paths_.RpLpRm );
        vis.draw_path_by_cvline( path_finder.all_possible_paths_.RpLmRp );
        vis.draw_path_by_cvline( path_finder.all_possible_paths_.RpLmRm );
        vis.draw_path_by_cvline( path_finder.all_possible_paths_.RmLpRp );
        vis.draw_path_by_cvline( path_finder.all_possible_paths_.RmLpRm );
        vis.draw_path_by_cvline( path_finder.all_possible_paths_.RmLmRp );
        vis.draw_path_by_cvline( path_finder.all_possible_paths_.RmLmRm );

        vis.draw_path_by_cvline( path_finder.all_possible_paths_.LpRupLumRm );
        vis.draw_path_by_cvline( path_finder.all_possible_paths_.LmRumLupRp );
        vis.draw_path_by_cvline( path_finder.all_possible_paths_.RpLupRumLm );
        vis.draw_path_by_cvline( path_finder.all_possible_paths_.RmLumRupLp );

        vis.draw_path_by_cvline( path_finder.all_possible_paths_.LpRumLumRp );
        vis.draw_path_by_cvline( path_finder.all_possible_paths_.LmRupLupRm );
        vis.draw_path_by_cvline( path_finder.all_possible_paths_.RpLumRumLp );
        vis.draw_path_by_cvline( path_finder.all_possible_paths_.RmLupRupLm );

        vis.draw_path_by_cvline( path_finder.all_possible_paths_.LpRm90SmLm );
        vis.draw_path_by_cvline( path_finder.all_possible_paths_.LmRp90SpLp );
        vis.draw_path_by_cvline( path_finder.all_possible_paths_.RpLm90SmRm );
        vis.draw_path_by_cvline( path_finder.all_possible_paths_.RmLp90SpRp );

        vis.draw_path_by_cvline( path_finder.all_possible_paths_.LpRm90SmRm );
        vis.draw_path_by_cvline( path_finder.all_possible_paths_.LmRp90SpRp );
        vis.draw_path_by_cvline( path_finder.all_possible_paths_.RpLm90SmLm );
        vis.draw_path_by_cvline( path_finder.all_possible_paths_.RmLp90SpLp );

        vis.draw_path_by_cvline( path_finder.all_possible_paths_.LmSmRm90Lp );
        vis.draw_path_by_cvline( path_finder.all_possible_paths_.LpSpRp90Lm );
        vis.draw_path_by_cvline( path_finder.all_possible_paths_.RmSmLm90Rp );
        vis.draw_path_by_cvline( path_finder.all_possible_paths_.RpSpLp90Rm );

        vis.draw_path_by_cvline( path_finder.all_possible_paths_.RmSmRm90Lp );
        vis.draw_path_by_cvline( path_finder.all_possible_paths_.RpSpRp90Lm );
        vis.draw_path_by_cvline( path_finder.all_possible_paths_.LmSmLm90Rp );
        vis.draw_path_by_cvline( path_finder.all_possible_paths_.LpSpLp90Rm );

        vis.draw_path_by_cvline( path_finder.all_possible_paths_.LpRm90SmLm90Rp );
        vis.draw_path_by_cvline( path_finder.all_possible_paths_.LmRp90SpLp90Rm );
        vis.draw_path_by_cvline( path_finder.all_possible_paths_.RpLm90SmRm90Lp );
        vis.draw_path_by_cvline( path_finder.all_possible_paths_.RmLp90SpRp90Lm );


        vis.show(manual);

        loop_count ++;
    }
    


    // while (goal_direction < 2*M_PI )
    // {
    //     goal_pose[0] = start_pose[0] + distance * cos(goal_direction);
    //     goal_pose[1] = start_pose[1] + distance * sin(goal_direction);
    //     goal_pose[2] = goal_direction * 2.0 ;
    //     goal_direction += pose2_pose_direction_step;

    //     path_finder.setup( start_pose, goal_pose);
    //     path_finder.search();

    //     vis.init();

    //     vis.draw_vector_on_map(start_pose);
    //     vis.draw_vector_on_map( goal_pose);

    //     // vis.draw_path_by_cvline( path_finder.all_possible_paths_.LpSpLp );
    //     // vis.draw_path_by_cvline( path_finder.all_possible_paths_.LpSpRp );
    //     // vis.draw_path_by_cvline( path_finder.all_possible_paths_.LmSmLm );
    //     // vis.draw_path_by_cvline( path_finder.all_possible_paths_.LmSmRm );

    //     // vis.draw_path_by_cvline( path_finder.all_possible_paths_.RpSpLp );
    //     // vis.draw_path_by_cvline( path_finder.all_possible_paths_.RpSpRp );
    //     // vis.draw_path_by_cvline( path_finder.all_possible_paths_.RmSmRm );
    //     // vis.draw_path_by_cvline( path_finder.all_possible_paths_.RmSmLm );

    //     // vis.draw_path_by_cvline( path_finder.all_possible_paths_.LpRmL );
    //     // vis.draw_path_by_cvline( path_finder.all_possible_paths_.LmRpLm );
    //     // vis.draw_path_by_cvline( path_finder.all_possible_paths_.RpLmRp );
    //     // vis.draw_path_by_cvline( path_finder.all_possible_paths_.RmLpRm );

    //     // vis.draw_path_by_cvline( path_finder.all_possible_paths_.LpRmLm );
    //     // vis.draw_path_by_cvline( path_finder.all_possible_paths_.LmRpLp );
    //     // vis.draw_path_by_cvline( path_finder.all_possible_paths_.RpLmRm );
    //     // vis.draw_path_by_cvline( path_finder.all_possible_paths_.RmLpRp );

    //     vis.draw_path_by_cvline( path_finder.all_possible_paths_.LpRupLumRm );
    //     vis.draw_path_by_cvline( path_finder.all_possible_paths_.LmRumLupRp );
    //     vis.draw_path_by_cvline( path_finder.all_possible_paths_.RpLupRumLm );
    //     vis.draw_path_by_cvline( path_finder.all_possible_paths_.RmLumRupLp );

    //     vis.show(manual);
        
    // }
    
}
