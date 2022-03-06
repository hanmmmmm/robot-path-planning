
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



Dubins::Dubins(  )
{
    

    all_path_colors["LSL"] = cv::Scalar(0,255,0);
    all_path_colors["RSR"] = cv::Scalar(255,0,0);
    all_path_colors["RSL"] = cv::Scalar(0,0,255);
    all_path_colors["LSR"] = cv::Scalar(255,255,0);
    all_path_colors["LRL"] = cv::Scalar(0,155,255);
    all_path_colors["RLR"] = cv::Scalar(255,0,255);

    // cv::threshold(map_fine.clone(), map_collision, 100, 255, 1);

    // cv::Mat erode_element = cv::getStructuringElement(0, cv::Size( 2*map_inflation_size+1, 2*map_inflation_size+1), cv::Point(map_inflation_size,map_inflation_size));
    // cv::erode(map_fine, map_fine, erode_element);
    // // cv::imshow("map fine", map_collision);

    

    // show_resize_img(&map_for_view, 1.0, 0, window_name);

}




Dubins::~Dubins()
{
}


void Dubins::setup(int startnode[], int goalnode[], float s_angle, float g_angle, cv::Mat map)
{
    cv::resize(map, map_fine, cv::Size(), fine_ratio, fine_ratio, cv::INTER_NEAREST);

    map_fine.setTo( cv::Scalar(255,255,255) );

    map_for_view = map_fine.clone();

    v_l = 2*turning_raius*sin(angular_step_size/2) ;

    fine_map_width = map_fine.size().width;
    fine_map_height = map_fine.size().height;

    start_pose[0] = startnode[0];
    start_pose[1] = startnode[1];
    start_pose[2] = s_angle;

    goal_pose[0] = goalnode[0];
    goal_pose[1] = goalnode[1];
    goal_pose[2] = g_angle;

    draw_start_goal_on_map( 60 );

    // show_resize_img(&map_for_view, 1.0, 0, window_name);

}


// void Dubins::search( std::array<float,3> start_pose, std::array<float,3> goal_pose)
void Dubins::search( bool last   )
{
    map_for_view = map_fine.clone();
    all_path_result.clear();

    // show_resize_img(&map_for_view, 1.0, 40, "46546");

    float distance = compute_h_cost_Euclidean(start_pose, goal_pose);
    std::vector<float> path_angle;
    if(distance > 2*turning_raius)
    {

        path_angle.clear();
        get_path(start_pose, goal_pose, "RSL", path_angle);

        path_angle.clear();
        get_path(start_pose, goal_pose, "LSR", path_angle);

        path_angle.clear();
        get_path(start_pose, goal_pose, "LSL", path_angle);

        path_angle.clear();
        get_path(start_pose, goal_pose, "RSR", path_angle);

        if(distance < 4*turning_raius)
        {
            path_angle.clear();
            get_path(start_pose, goal_pose, "LRL", path_angle);

            path_angle.clear();
            get_path(start_pose, goal_pose, "RLR", path_angle);
        }


    }

    int type_counter = 0;
    for(auto curve : all_path_result)
    {
        std::string curve_name = curve.first;
        for(auto ps : curve.second.path_steps)
        {
            cv::circle(map_for_view, cv::Point(ps.x, ps.y), 2, all_path_colors[curve_name],  -1);

        }

        std::ostringstream str;
        str << curve_name << " " << curve.second.path_length;
        
        cv::putText(map_for_view , str.str().substr(0,8) , cv::Point(15,30 + type_counter*30),cv::FONT_HERSHEY_DUPLEX,1, all_path_colors[curve_name] ,2,false);


        std::cout << curve_name << "  cost: " << curve.second.path_length << std::endl;

        type_counter ++;
    }

    draw_start_goal_on_map( 60 );

    if(last){
        show_resize_img(&map_for_view, 1.0, 0, window_name);
    }
    else
    {
        show_resize_img(&map_for_view, 1.0, 40, window_name);
    }
    

    // show_resize_img(&map_fine, 1.0, 40, "46546");

    
}


void Dubins::get_samples_L( const float target_angle_change, float robot_yaw, float& robotx, float& roboty, const std::string path_type  )
{
    float theta_change = 0;
    while (theta_change < target_angle_change)
        {
            float dx = v_l * cos( robot_yaw + angular_step_size/2);
            float dy = v_l * sin( robot_yaw + angular_step_size/2);
            robotx = robotx + (dx);
            roboty = roboty + (dy);
            theta_change += angular_step_size;
            robot_yaw += angular_step_size ;

            // cv::circle(map_for_view, cv::Point(robotx, roboty), 1, cv::Scalar(0,255,0),  -1);

            posePerSample pose(robotx, roboty, robot_yaw);
            all_path_result[path_type].path_steps.push_back(pose);

            // std::cout << " raius " << turning_raius ;
            // std::cout << "  robot_theta "  << int(robot_theta*180.0/M_PI) ;
            // std::cout << "  x " << robot_x << " y " << robot_y ;
            // std::cout << "  angular_step_size " << angular_step_size  ;
            // std::cout << "  theta_change " << theta_change  ;
            // std::cout << "  dx " << dx << " dy " << dy << std::endl;
            // std::cout << robot_theta*180.0/M_PI << " " << robot_x << " " << robot_y << " " << v_l  << " " << dx << " " << dy << std::endl;
        }
}


void Dubins::get_samples_R( const float target_angle_change, float robot_yaw, float& robotx, float& roboty, const std::string path_type  )
{
    float theta_change = 0;
    while (theta_change < target_angle_change)
        {
            float dx = v_l * cos( robot_yaw + angular_step_size/2);
            float dy = v_l * sin( robot_yaw + angular_step_size/2);
            robotx = robotx + (dx);
            roboty = roboty + (dy);
            theta_change += angular_step_size;
            robot_yaw -= angular_step_size ;

            // cv::circle(map_for_view, cv::Point(robotx, roboty), 1, cv::Scalar(0,255,0),  -1);

            posePerSample pose(robotx, roboty, robot_yaw);
            all_path_result[path_type].path_steps.push_back(pose);

            // std::cout << " raius " << turning_raius ;
            // std::cout << "  robot_theta "  << int(robot_theta*180.0/M_PI) ;
            // std::cout << "  x " << robot_x << " y " << robot_y ;
            // std::cout << "  angular_step_size " << angular_step_size  ;
            // std::cout << "  theta_change " << theta_change  ;
            // std::cout << "  dx " << dx << " dy " << dy << std::endl;
            // std::cout << robot_theta*180.0/M_PI << " " << robot_x << " " << robot_y << " " << v_l  << " " << dx << " " << dy << std::endl;
        }
}



void Dubins::get_samples_S( const float target_angle_change, float robot_yaw, float& robotx, float& roboty, const std::string path_type  )
{
    float theta_change = 0;
    while (theta_change < target_angle_change)
        {
            robotx +=  linear_step_size * cos(robot_yaw);
            roboty +=  linear_step_size * sin(robot_yaw);
            theta_change += linear_step_size;
            posePerSample pose(robotx, roboty, robot_yaw);
            all_path_result[path_type].path_steps.push_back(pose);

        }
}





void Dubins::get_path( std::array<float,3> start_pose, std::array<float,3> goal_pose , std::string path_type, std::vector<float>& path_angle)
{
    float dyy = goal_pose[1] - start_pose[1];
    float dxx = goal_pose[0] - start_pose[0];
    float theta = std::atan2(dyy,dxx);
    float D = sqrt( dxx*dxx + dyy*dyy );
    float d = D/turning_raius;
    float alpha = rectify_angle_rad( start_pose[2] - theta );
    float beta  = rectify_angle_rad( goal_pose[2] - theta );


    float robot_x     = start_pose[0] ;
    float robot_y     = start_pose[1] ;
    float robot_theta = start_pose[2] ;

    float theta_change = 0;


    

    if( path_type == "LSL")
    {
        float path_length = 0;
        compute_LSL_path(alpha, beta, d, path_angle, path_length);
        all_path_result[path_type].path_length = path_length;

        get_samples_L( path_angle[0], robot_theta, robot_x, robot_y, path_type);
        robot_theta = start_pose[2] + path_angle[0];
        get_samples_S( path_angle[1], robot_theta, robot_x, robot_y, path_type);
        get_samples_L( path_angle[2], robot_theta, robot_x, robot_y, path_type);
    }
    else if( path_type == "RSR")
    {
        float path_length = 0;
        compute_RSR_path(alpha, beta, d, path_angle, path_length);
        all_path_result[path_type].path_length = path_length;

        get_samples_R( path_angle[0], robot_theta, robot_x, robot_y, path_type);
        robot_theta = start_pose[2] - path_angle[0];
        get_samples_S( path_angle[1], robot_theta, robot_x, robot_y, path_type);
        get_samples_R( path_angle[2], robot_theta, robot_x, robot_y, path_type);

    }    

    else if( path_type == "LSR")
    {
        float path_length = 0;
        compute_LSR_path(alpha, beta, d, path_angle, path_length);
        all_path_result[path_type].path_length = path_length;

        get_samples_L( path_angle[0], robot_theta, robot_x, robot_y, path_type);
        robot_theta = start_pose[2] + path_angle[0];
        get_samples_S( path_angle[1], robot_theta, robot_x, robot_y, path_type);
        get_samples_R( path_angle[2], robot_theta, robot_x, robot_y, path_type);
    }    


    else if( path_type == "RSL")
    {
        float path_length = 0;
        compute_RSL_path(alpha, beta, d, path_angle, path_length);
        all_path_result[path_type].path_length = path_length;

        get_samples_R( path_angle[0], robot_theta, robot_x, robot_y, path_type);
        robot_theta = start_pose[2] - path_angle[0];
        get_samples_S( path_angle[1], robot_theta, robot_x, robot_y, path_type);
        get_samples_L( path_angle[2], robot_theta, robot_x, robot_y, path_type);
    }    

    else if( path_type == "LRL")
    {
        float path_length = 0;
        compute_LRL_path(alpha, beta, d, path_angle, path_length);
        all_path_result[path_type].path_length = path_length;

        get_samples_L( path_angle[0], robot_theta, robot_x, robot_y, path_type);
        robot_theta = start_pose[2] + path_angle[0];
        get_samples_R( path_angle[1], robot_theta, robot_x, robot_y, path_type);
        robot_theta -= path_angle[1];
        get_samples_L( path_angle[2], robot_theta, robot_x, robot_y, path_type);
    }    

    else if( path_type == "RLR")
    {
        float path_length = 0;
        compute_RLR_path(alpha, beta, d, path_angle, path_length);
        all_path_result[path_type].path_length = path_length;

        get_samples_R( path_angle[0], robot_theta, robot_x, robot_y, path_type);
        robot_theta = start_pose[2] - path_angle[0];
        get_samples_L( path_angle[1], robot_theta, robot_x, robot_y, path_type);
        robot_theta += path_angle[1];
        get_samples_R( path_angle[2], robot_theta, robot_x, robot_y, path_type);
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







