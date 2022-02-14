
#include <iostream>
#include "dubins.h"
#include <string>
#include <stdexcept>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <limits>
#include <math.h>

#include "utils/img_io.h"
#include "utils/bresenham.h"
#include "utils/math_tools.h"



Dubins::Dubins(int startnode[], int goalnode[], float s_angle, float g_angle, cv::Mat map)
{
    cv::resize(map, map_fine, cv::Size(), fine_ratio, fine_ratio, cv::INTER_NEAREST);

    map_fine.setTo( cv::Scalar(255,255,255) );

    map_for_view = map_fine.clone();

    

    cv::threshold(map_fine.clone(), map_collision, 100, 255, 1);

    cv::Mat erode_element = cv::getStructuringElement(0, cv::Size( 2*map_inflation_size+1, 2*map_inflation_size+1), cv::Point(map_inflation_size,map_inflation_size));
    cv::erode(map_fine, map_fine, erode_element);
    // cv::imshow("map fine", map_collision);

    fine_map_width = map_fine.size().width;
    fine_map_height = map_fine.size().height;

    start_pose[0] = startnode[0];
    start_pose[1] = startnode[1];
    start_pose[2] = s_angle;

    goal_pose[0] = goalnode[0];
    goal_pose[1] = goalnode[1];
    goal_pose[2] = g_angle;

    draw_start_goal_on_map( 35 );

    std::array<float,2> center_xy ;
    // float angle_on_circle;




    compute_circle_center(start_pose, "L", turning_raius, center_xy);
    map_for_view.at<cv::Vec3b>(center_xy[1],center_xy[0]) = cv::Vec3b(255,0,0);

    // compute_circle_center(start_pose, "R", turning_raius, center_xy, angle_on_circle);
    // map_for_view.at<cv::Vec3b>(center_xy[1],center_xy[0]) = cv::Vec3b(255,200,0);

    compute_circle_center(goal_pose, "L", turning_raius, center_xy);
    map_for_view.at<cv::Vec3b>(center_xy[1],center_xy[0]) = cv::Vec3b(0,50,250);

    // compute_circle_center(goal_pose, "R", turning_raius, center_xy, angle_on_circle);
    // map_for_view.at<cv::Vec3b>(center_xy[1],center_xy[0]) = cv::Vec3b(0,150,250);

    show_resize_img(&map_for_view, 1.0, 0, window_name);

}




Dubins::~Dubins()
{
}



// void Dubins::search( std::array<float,3> start_pose, std::array<float,3> goal_pose)
void Dubins::search(   )
{
    float distance = compute_h_cost_Euclidean(start_pose, goal_pose);
    std::vector<float> path_angle;
    if(distance > 4*turning_raius)
    {
        path_angle.clear();
        get_path(start_pose, goal_pose, "LSL", path_angle);

        path_angle.clear();
        get_path(start_pose, goal_pose, "RSR", path_angle);

    }

    show_resize_img(&map_for_view, 1.0, 0, window_name);

    
}


void Dubins::get_path( std::array<float,3> start_pose, std::array<float,3> goal_pose , std::string path_type, std::vector<float>& path_angle)
{
    float dy = goal_pose[1] - start_pose[1];
    float dx = goal_pose[0] - start_pose[0];
    float theta = std::atan2(dy,dx);
    float D = sqrt( dx*dx + dy*dy );
    float d = D/turning_raius;
    float alpha = rectify_angle_rad( start_pose[2] - theta );
    float beta  = rectify_angle_rad( goal_pose[2] - theta );

    std::array<float,3> path_1_exit_pose, path_2_exit_pose;

    float robot_x = ( start_pose[0] );
    float robot_y = ( start_pose[1] );
    float robot_theta = start_pose[2];
    float theta_change = 0;
    float theta_step = 0.05;

    std::array<float,2> center_xy ;
    // float angle_on_circle;
    compute_circle_center(start_pose, "L", turning_raius, center_xy);


    if( path_type == "LSL")
    {
        compute_LSL_path(alpha, beta, d, path_angle);

        path_angle[0] = rectify_angle_rad(path_angle[0]);
        path_angle[1] = path_angle[1]*turning_raius;
        path_angle[2] = rectify_angle_rad(path_angle[2]);

        std::cout << path_angle[0]*180.0/M_PI << " " << path_angle[1] << " " << path_angle[2]*180.0/M_PI << std::endl;

        // path_1_exit_pose[2] = start_pose[2] + path_angle[0];
        // path_1_exit_pose[0] = start_pose[0] + (sin(start_pose[2] + path_angle[2]) - sin(start_pose[2]))*turning_raius;
        // path_1_exit_pose[1] = start_pose[1] - (cos(start_pose[2] + path_angle[2]) + cos(start_pose[2]))*turning_raius;

        // path_2_exit_pose[2] = path_1_exit_pose[2];
        // path_2_exit_pose[0] = path_1_exit_pose[0] + (path_angle[1] * cos(path_2_exit_pose[2]))*turning_raius;
        // path_2_exit_pose[1] = path_1_exit_pose[1] + (path_angle[1] * sin(path_2_exit_pose[2]))*turning_raius;

        // cv::circle(map_for_view, cv::Point(center_xy[0], center_xy[1]), turning_raius, cv::Scalar(0,155,60),  2);

        

        while (theta_change < path_angle[0])
        {
            float v_l = 2*turning_raius*sin(theta_step/2) ;
            float dx = v_l * cos( robot_theta + theta_step/2);
            float dy = v_l * sin( robot_theta + theta_step/2);
            robot_x = robot_x + (dx);
            robot_y = robot_y + (dy);
            theta_change += theta_step;
            robot_theta += theta_step ;

            cv::circle(map_for_view, cv::Point(robot_x, robot_y), 1, cv::Scalar(0,255,0),  -1);

            // std::cout << " raius " << turning_raius ;
            // std::cout << "  robot_theta "  << int(robot_theta*180.0/M_PI) ;
            // std::cout << "  x " << robot_x << " y " << robot_y ;
            // std::cout << "  theta_step " << theta_step  ;
            // std::cout << "  theta_change " << theta_change  ;
            // std::cout << "  dx " << dx << " dy " << dy << std::endl;
            // std::cout << robot_theta*180.0/M_PI << " " << robot_x << " " << robot_y << " " << v_l  << " " << dx << " " << dy << std::endl;
        }

        theta_change = 0;
        theta_step = 2;
        // float path_1_exit_x = robot_x;
        // float path_1_exit_y = robot_y;
        while (theta_change < path_angle[1])
        {
            float robot_theta = start_pose[2] + path_angle[0];
            // robot_x = path_1_exit_x + theta_change * cos(robot_theta);
            // robot_y = path_1_exit_y + theta_change * sin(robot_theta);

            robot_x +=  theta_step * cos(robot_theta);
            robot_y +=  theta_step * sin(robot_theta);
            theta_change += theta_step;

            cv::circle(map_for_view, cv::Point(robot_x, robot_y), 1, cv::Scalar(0,255,0),  -1);

            // std::cout << "  x " << robot_x << " y " << robot_y ;
            // std::cout << "  theta_step " << theta_step  ;
            // std::cout << "  theta_change " << theta_change  << std::endl;
        }


        theta_change = 0;
        theta_step = 0.05;
        // float path_2_exit_x = robot_x;
        // float path_2_exit_y = robot_y;
        while (theta_change < path_angle[2])
        {
            float v_l = 2*turning_raius*sin(theta_step/2) ;
            float dx = v_l * cos( robot_theta + theta_step/2);
            float dy = v_l * sin( robot_theta + theta_step/2);
            robot_x = robot_x + (dx);
            robot_y = robot_y + (dy);
            theta_change += theta_step;
            robot_theta += theta_step ;
            cv::circle(map_for_view, cv::Point(robot_x, robot_y), 1, cv::Scalar(0,255,0),  -1);
        }
        
        

    }
    else if( path_type == "RSR")
    {
        compute_RSR_path(alpha, beta, d, path_angle);
        path_angle[0] = rectify_angle_rad(path_angle[0]);
        path_angle[1] = path_angle[1]*turning_raius;
        path_angle[2] = rectify_angle_rad(path_angle[2]);

        while (theta_change < path_angle[0])
        {
            float v_l = 2*turning_raius*sin(theta_step/2) ;
            float dx = v_l * cos( robot_theta + theta_step/2);
            float dy = v_l * sin( robot_theta + theta_step/2);
            robot_x = robot_x + (dx);
            robot_y = robot_y + (dy);
            theta_change += theta_step;
            robot_theta -= theta_step ;
            cv::circle(map_for_view, cv::Point(robot_x, robot_y), 1, cv::Scalar(0,0,255),  -1);

            // std::cout << " raius " << turning_raius ;
            // std::cout << "  robot_theta "  << int(robot_theta*180.0/M_PI) ;
            // std::cout << "  x " << robot_x << " y " << robot_y ;
            // std::cout << "  theta_step " << theta_step  ;
            // std::cout << "  theta_change " << theta_change  ;
            // std::cout << "  dx " << dx << " dy " << dy << std::endl;
            // std::cout << robot_theta*180.0/M_PI << " " << robot_x << " " << robot_y << " " << v_l  << " " << dx << " " << dy << std::endl;
        }

        theta_change = 0;
        theta_step = 2;
        float path_1_exit_x = robot_x;
        float path_1_exit_y = robot_y;
        while (theta_change < path_angle[1])
        {
            float robot_theta = start_pose[2] - path_angle[0];
            robot_x = path_1_exit_x + theta_change * cos(robot_theta);
            robot_y = path_1_exit_y + theta_change * sin(robot_theta);
            theta_change += theta_step;
            cv::circle(map_for_view, cv::Point(robot_x, robot_y), 1, cv::Scalar(0,0,255),  -1);
        }


        theta_change = 0;
        theta_step = 0.05;
        float path_2_exit_x = robot_x;
        float path_2_exit_y = robot_y;
        while (theta_change < path_angle[2])
        {
            float v_l = 2*turning_raius*sin(theta_step/2) ;
            float dx = v_l * cos( robot_theta + theta_step/2);
            float dy = v_l * sin( robot_theta + theta_step/2);
            robot_x = robot_x + (dx);
            robot_y = robot_y + (dy);
            theta_change += theta_step;
            robot_theta -= theta_step ;
            cv::circle(map_for_view, cv::Point(robot_x, robot_y), 1, cv::Scalar(0,0,255),  -1);
        }


    }    



}




void Dubins::compute_circle_center(std::array<float,3> robot_pose, std::string LorR, int radius, std::array<float,2>& center )
{
    // float a = robot_pose[2];

    if(LorR == "L"){
        robot_pose[2] += M_PI/2.0;
    }
    else if(LorR == "R")
    {
        robot_pose[2] -= M_PI/2.0;
    }
    else
    {
        throw std::runtime_error("Invalid Left/Right request !!!");
    }

    robot_pose[0] += cos(robot_pose[2]) * radius;
    robot_pose[1] += sin(robot_pose[2]) * radius;

    center[0] = robot_pose[0];
    center[1] = robot_pose[1];
    // robot_angle = robot_pose[2];
}









// void Dubins::get_path()
// {
//     std::array<int, 3> get_path_curr_node = fine_to_grid(close_goal_pose, fine_ratio, angle_resolution);
//     std::cout << "get_path" << std::endl;
//     cv::Scalar line_color(0, 0, 255);
//     // path.push_front(goal_grid);
//     cv::line(map_for_view, cv::Point(goal_pose[0],goal_pose[1]), cv::Point(close_goal_pose[0],close_goal_pose[1]), line_color, 3);
//     cv::circle(map_for_view, cv::Point(close_goal_pose[0],close_goal_pose[1]), 4, cv::Scalar(255, 0, 0), -1, 8, 0);
//     while (path.front() != start_grid)
//     {
//         path.push_front(get_path_curr_node);
//         std::array<float, 3> prev_point = grid_fine_pose[get_path_curr_node];
//         get_path_curr_node = grid_parent[get_path_curr_node];
//         std::array<float, 3> curr_point = grid_fine_pose[get_path_curr_node];

//         cv::Point p1(prev_point[0], prev_point[1]);
//         cv::Point p2(curr_point[0], curr_point[1]);

//         cv::line(map_for_view, p1, p2, line_color, 3);
//         // draw_arc(grid_fine_pose[get_path_curr_node], grid_fine_pose[ path.front()], turning_raius[1], turning_angle[1], motion, 2, cv::Scalar(0,50,255));
//         cv::circle(map_for_view, p2, 4, cv::Scalar(255, 0, 0), -1, 8, 0);
//     }
//     // cv::line(map_for_view, p1, p2, line_color, 3);
//     // cv::circle(map_for_view, p2, 4, cv::Scalar(255, 0, 0), -1, 8, 0);

//     draw_start_goal_on_map();
//     show_resize_img(&map_for_view, 1.0, 0, window_name);
// }


// void Dubins::check_if_reach_goal(std::array<float, 3> node_pose,std::array<float, 3> in_goal_pose, std::array<int, 3> in_curr_grid){
//     if (compute_h_cost_Euclidean(node_pose, in_goal_pose) < 12)
//     {
//         if (abs(node_pose[2] - in_goal_pose[2]) < 0.2)
//         {
//             FLAG_reach_goal = true;
//             close_goal_angle = node_pose[2];
//             close_goal_pose = node_pose;
//             grid_parent[goal_grid] = in_curr_grid;
//             grid_fine_pose[goal_grid] = node_pose;
//             grid_status[goal_grid][2] = 2;
//         }
//     }
// }

