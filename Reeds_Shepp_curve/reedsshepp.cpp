
#include <iostream>
#include "reedsshepp.h"
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



ReedsSheppClass::ReedsSheppClass(  )
{
    
    all_path_colors["LpSpLp"] = cv::Scalar(0,255,0);
    all_path_colors["LpSpRp"] = cv::Scalar(255,0,0);
    all_path_colors["LmSmLm"] = cv::Scalar(0,0,255);
    all_path_colors["LmSmRm"] = cv::Scalar(0,200,100);
    all_path_colors["RpSpRp"] = cv::Scalar(80,0,155);
    all_path_colors["RpSpLp"] = cv::Scalar(255,0,0);
    all_path_colors["RmSmRm"] = cv::Scalar(0,0,255);
    all_path_colors["RmSmLm"] = cv::Scalar(0,200,100);

    all_path_colors["LpRmL"] = cv::Scalar(0,255,0);
    all_path_colors["LmRpLm"] = cv::Scalar(255,0,0);
    all_path_colors["RpLmRp"] = cv::Scalar(0,0,255);
    all_path_colors["RmLpRm"] = cv::Scalar(0,200,100);
    all_path_colors["LpRmLm"] = cv::Scalar(80,0,155);
    all_path_colors["LmRpLp"] = cv::Scalar(255,0,0);
    all_path_colors["RpLmRm"] = cv::Scalar(0,0,255);
    all_path_colors["RmLpRp"] = cv::Scalar(0,200,100);




    // cv::threshold(map_fine.clone(), map_collision, 100, 255, 1);

    // cv::Mat erode_element = cv::getStructuringElement(0, cv::Size( 2*map_inflation_size+1, 2*map_inflation_size+1), cv::Point(map_inflation_size,map_inflation_size));
    // cv::erode(map_fine, map_fine, erode_element);
    // // cv::imshow("map fine", map_collision);

    

    // show_resize_img(&map_for_view, 1.0, 0, window_name);

    std::cout << "ZERO " << ZERO << std::endl;
    std::cout << "RS_EPS " << RS_EPS << std::endl;


    // double t,u,v;
    // LpSpLp(2,1,0.3,t,u,v);
    // std::cout << "t " << t << std::endl;
    // std::cout << "u " << u << std::endl;
    // std::cout << "v " << v << std::endl;

}




ReedsSheppClass::~ReedsSheppClass()
{
}


void ReedsSheppClass::setup(int startnode[], int goalnode[], float s_angle, float g_angle, cv::Mat map)
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


void ReedsSheppClass::polar(double x, double y, double &radius_ro, double &theta)
{
    radius_ro = sqrt(x*x + y*y);
    theta = atan2(y, x);
}

double ReedsSheppClass::mod2pi(double angle_radian)
{
    double v = fmod(angle_radian, pi2_);
    while (v < - M_PI){
        v += pi2_;
    }
    while (v > M_PI){
        v -= pi2_;
    }
    return v;
}


// void ReedsSheppClass::search( std::array<float,3> start_pose, std::array<float,3> goal_pose)
void ReedsSheppClass::search( bool last   )
{
    map_for_view = map_fine.clone();
    all_path_result.clear();

    // show_resize_img(&map_for_view, 1.0, 40, "46546");

    std::vector<float> path_angle;
    bool valid;

    // path_angle.clear(); 
    // valid = false;
    // get_path(start_pose, goal_pose, "LpSpLp", path_angle, valid);

    // valid = false;
    // path_angle.clear();
    // get_path(start_pose, goal_pose, "LpSpRp", path_angle, valid);

    // valid = false;
    // path_angle.clear();
    // get_path(start_pose, goal_pose, "LmSmLm", path_angle, valid);

    // valid = false;
    // path_angle.clear();
    // get_path(start_pose, goal_pose, "LmSmRm", path_angle, valid);

    // valid = false;
    // path_angle.clear();
    // get_path(start_pose, goal_pose, "RpSpRp", path_angle, valid);

    // valid = false;
    // path_angle.clear();
    // get_path(start_pose, goal_pose, "RpSpLp", path_angle, valid);

    // valid = false;
    // path_angle.clear();
    // get_path(start_pose, goal_pose, "RmSmRm", path_angle, valid);

    // valid = false;
    // path_angle.clear();
    // get_path(start_pose, goal_pose, "RmSmLm", path_angle, valid);

    valid = false;
    path_angle.clear();
    get_path(start_pose, goal_pose, "LpRmL", path_angle, valid);

    valid = false;
    path_angle.clear();
    get_path(start_pose, goal_pose, "RpLmRp", path_angle, valid);




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

// 8.1 in paper 
void ReedsSheppClass::LpSpLp(double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid)
{
    polar( x-sin(phi), y-1+cos(phi), u, t );
    v = mod2pi(phi - t);
    L = std::abs(t) + std::abs(u) + std::abs(v) ;
    valid = true;
    // std::cout << "phi :" << phi << std::endl;
    // std::cout << "t :" << t << std::endl;
    // std::cout << "v :" << v << std::endl;
}

void ReedsSheppClass::LpSpRp(double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid)
{
    double t1, u1;
    polar(x + sin(phi), y-1.0-cos(phi), u1, t1);
    if(u1*u1 < 4.0){
        valid = false;
        return;
    }
    u = sqrt(u1*u1 - 4.);
    double theta = atan2(2., u);
    t = mod2pi(t1 + theta);
    v = mod2pi(t - phi);
    L = std::abs(t) + std::abs(u) + std::abs(v) ;
    if (t >= 0.0 && v >= 0.0) valid = true;
    else valid = false;
}


void ReedsSheppClass::LmSmLm(double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid){
    LpSpLp(-x,y,-phi,t,u,v,L,valid);
}

void ReedsSheppClass::LmSmRm(double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid){
    LpSpRp(-x,y,-phi,t,u,v,L,valid);
}

void ReedsSheppClass::RpSpRp(double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid){
    LpSpLp(x,-y,-phi,t,u,v,L,valid);
}

void ReedsSheppClass::RpSpLp(double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid){
    LpSpRp(x, -y,-phi,t,u,v,L,valid);
}

void ReedsSheppClass::RmSmRm(double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid){
    LpSpLp(-x,-y, phi,t,u,v,L,valid);
}

void ReedsSheppClass::RmSmLm(double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid){
    LpSpRp(-x,-y, phi,t,u,v,L,valid);
}

// void ReedsSheppClass::LmSmRm(double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid){
//     LpSpRp(-x,y,-phi,t,u,v,L,valid);
// }

void ReedsSheppClass::LpRmL(double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid){
    double xi = x - sin(phi); 
    double eta = y - 1. + cos(phi); 
    double u1, theta;
    polar(xi, eta, u1, theta);
    
    if (u1 > 4.0){
        valid = false;
    }
    else{
        u = -2.0 * asin( u1 / 4.0 );
        t = mod2pi(theta + 0.5 * u + M_PI);
        v = mod2pi(phi - t + u);
        if (t >= 0.0 && u <= 0.0){
            valid = true;
        }
        else{
            valid = false;
        }
    }
}


    

void ReedsSheppClass::LmRpLm(double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid){
    LpRmL(-x, y, -phi, t, u, v, L, valid);
}

void ReedsSheppClass::RpLmRp(double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid){
    LpRmL(x, -y, -phi, t, u, v, L, valid);
}

void ReedsSheppClass::RmLpRm(double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid){
    LpRmL(-x, -y, phi, t, u, v, L, valid);
}

// void ReedsSheppClass::LpRmLm(double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid){
//     double xb = x*cos(phi)+y*sin(phi);
//     double yb = x*sin(phi)-y*cos(phi);
//     LpRmL( xb, yb, phi, t, u, v, L, valid);
// }

// void ReedsSheppClass::LmRpLp(double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid){
//     double xb = x*cos(phi)+y*sin(phi);
//     double yb = x*sin(phi)-y*cos(phi);
//     LpRmL( -xb, yb, -phi, t, u, v, L, valid);
// }

// void ReedsSheppClass::RpLmRm(double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid){
//     double xb = x*cos(phi)+y*sin(phi);
//     double yb = x*sin(phi)-y*cos(phi);
//     LpRmL( xb, -yb, -phi, t, u, v, L, valid);
// }


// void ReedsSheppClass::RmLpRp(double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid){
//     double xb = x*cos(phi)+y*sin(phi);
//     double yb = x*sin(phi)-y*cos(phi);
//     LpRmL( -xb, -yb, phi, t, u, v, L, valid);
// }


void ReedsSheppClass::get_path( std::array<float,3> start_pose, std::array<float,3> goal_pose , std::string path_type, std::vector<float>& path_angle, bool& valid)
{
    float dy = goal_pose[1] - start_pose[1];
    float dx = goal_pose[0] - start_pose[0];
    double cos_theta1 = cos(start_pose[2]);
    double sin_theta1 = sin(start_pose[2]);

    // std::cout << "goal pose: " << goal_pose[0] << " " << goal_pose[1]  << " " << goal_pose[2] << std::endl;  

    float goal_x_rotated = (dx * cos_theta1 + dy * sin_theta1) / turning_raius;
    float goal_y_rotated = (-dx * sin_theta1 + dy * cos_theta1) / turning_raius;
    float goal_theta = rectify_angle_rad( goal_pose[2] - start_pose[2] );

    float robot_x     = start_pose[0] ;
    float robot_y     = start_pose[1] ;
    float robot_theta = start_pose[2] ;

    float theta_change = 0;


    if(path_type == "LpSpLp" ){
        double t,u,v, L; 
        LpSpLp(goal_x_rotated, goal_y_rotated,goal_theta, t,u,v,L,valid);
        if( valid ){
            std::cout << "t " << t << std::endl;
            std::cout << "u " << u << std::endl;
            std::cout << "v " << v << std::endl;

            get_samples_L( t, robot_theta, robot_x, robot_y, path_type);
            robot_theta = start_pose[2] + t;
            get_samples_S( u, robot_theta, robot_x, robot_y, path_type);

            get_samples_L( v, robot_theta, robot_x, robot_y, path_type);
        }
    }
    if(path_type == "LpSpRp" ){
        double t,u,v, L; 
        LpSpRp(goal_x_rotated, goal_y_rotated,goal_theta, t,u,v,L,valid);
        if( valid ){
            std::cout << "t " << t << std::endl;
            std::cout << "u " << u << std::endl;
            std::cout << "v " << v << std::endl;

            get_samples_L( t, robot_theta, robot_x, robot_y, path_type);
            robot_theta = start_pose[2] + t;
            get_samples_S( u, robot_theta, robot_x, robot_y, path_type);
            get_samples_R( v, robot_theta, robot_x, robot_y, path_type);
        }
        else{
            std::cout << "           Invalid pose for LpSpRp" << std::endl;
        }
    }
    if(path_type == "LmSmLm" ){
        double t,u,v, L; 
        LmSmLm(goal_x_rotated, goal_y_rotated,goal_theta, t,u,v,L,valid);
        if( valid ){
            std::cout << "t " << t << std::endl;
            std::cout << "u " << u << std::endl;
            std::cout << "v " << v << std::endl;

            get_samples_L( -t, robot_theta, robot_x, robot_y, path_type);
            robot_theta = start_pose[2] - t;
            get_samples_S( -u, robot_theta, robot_x, robot_y, path_type);

            get_samples_L( -v, robot_theta, robot_x, robot_y, path_type);
        }
    }

    if(path_type == "LmSmRm" ){
        double t,u,v, L; 
        LmSmRm(goal_x_rotated, goal_y_rotated,goal_theta, t,u,v,L,valid);
        if( valid ){
            std::cout << "t " << t << std::endl;
            std::cout << "u " << u << std::endl;
            std::cout << "v " << v << std::endl;

            get_samples_L( -t, robot_theta, robot_x, robot_y, path_type);
            robot_theta = start_pose[2] - t;
            get_samples_S( -u, robot_theta, robot_x, robot_y, path_type);
            get_samples_R( -v, robot_theta, robot_x, robot_y, path_type);
        }
    }

    if(path_type == "RpSpRp" ){
        double t,u,v, L; 
        RpSpRp(goal_x_rotated, goal_y_rotated,goal_theta, t,u,v,L,valid);
        if( valid ){
            std::cout << "t " << t << std::endl;
            std::cout << "u " << u << std::endl;
            std::cout << "v " << v << std::endl;

            get_samples_R( t, robot_theta, robot_x, robot_y, path_type);
            robot_theta = start_pose[2] - t;
            get_samples_S( u, robot_theta, robot_x, robot_y, path_type);
            get_samples_R( v, robot_theta, robot_x, robot_y, path_type);
        }
    }

    if(path_type == "RpSpLp" ){
        double t,u,v, L; 
        RpSpLp(goal_x_rotated, goal_y_rotated,goal_theta, t,u,v,L,valid);
        if( valid ){
            std::cout << "t " << t << std::endl;
            std::cout << "u " << u << std::endl;
            std::cout << "v " << v << std::endl;

            get_samples_R( t, robot_theta, robot_x, robot_y, path_type);
            robot_theta = start_pose[2] - t;
            get_samples_S( u, robot_theta, robot_x, robot_y, path_type);
            get_samples_L( v, robot_theta, robot_x, robot_y, path_type);
        }
    }

    if(path_type == "RmSmRm" ){
        double t,u,v, L; 
        RmSmRm(goal_x_rotated, goal_y_rotated,goal_theta, t,u,v,L,valid);
        if( valid ){
            std::cout << "t " << t << std::endl;
            std::cout << "u " << u << std::endl;
            std::cout << "v " << v << std::endl;

            get_samples_R( -t, robot_theta, robot_x, robot_y, path_type);
            robot_theta = start_pose[2] + t;
            get_samples_S( -u, robot_theta, robot_x, robot_y, path_type);
            get_samples_R( -v, robot_theta, robot_x, robot_y, path_type);
        }
    }

    if(path_type == "RmSmLm" ){
        double t,u,v, L; 
        RmSmLm(goal_x_rotated, goal_y_rotated,goal_theta, t,u,v,L,valid);
        if( valid ){
            std::cout << "t " << t << std::endl;
            std::cout << "u " << u << std::endl;
            std::cout << "v " << v << std::endl;

            get_samples_R( -t, robot_theta, robot_x, robot_y, path_type);
            robot_theta = start_pose[2] + t;
            get_samples_S( -u, robot_theta, robot_x, robot_y, path_type);
            get_samples_L( -v, robot_theta, robot_x, robot_y, path_type);
        }
    }

    if(path_type == "LpRmL" ){
        double t,u,v, L; 
        LpRmL(goal_x_rotated, goal_y_rotated,goal_theta, t,u,v,L,valid);
        if( valid ){
            std::cout << "t " << t << std::endl;
            std::cout << "u " << u << std::endl;
            std::cout << "v " << v << std::endl;

            get_samples_L( t, robot_theta, robot_x, robot_y, path_type);
            // robot_theta = start_pose[2] + t;
            std::cout << "robot_theta " << robot_theta << std::endl;
            get_samples_R( u, robot_theta, robot_x, robot_y, path_type);
            // robot_theta += u;
            std::cout << "robot_theta " << robot_theta << std::endl;
            get_samples_L( v, robot_theta, robot_x, robot_y, path_type);
        }
    }

    if(path_type == "RpLmRp" ){
        double t,u,v, L; 
        RpLmRp(goal_x_rotated, goal_y_rotated,goal_theta, t,u,v,L,valid);
        if( valid ){
            std::cout << "t " << t << std::endl;
            std::cout << "u " << u << std::endl;
            std::cout << "v " << v << std::endl;

            get_samples_R( t, robot_theta, robot_x, robot_y, path_type);
            // robot_theta = start_pose[2] + t;
            std::cout << "robot_theta " << robot_theta << std::endl;
            get_samples_L( u, robot_theta, robot_x, robot_y, path_type);
            // robot_theta += u;
            std::cout << "robot_theta " << robot_theta << std::endl;
            get_samples_R( v, robot_theta, robot_x, robot_y, path_type);
        }
    }


}


void ReedsSheppClass::get_samples_L( const float target_angle_change, float& robot_yaw, float& robotx, float& roboty, const std::string path_type  )
{
    double init_yaw = robot_yaw;
    float theta_change = 0;
    double speed = 0.0;
    double ang_step_size_local = 0.0;
    if( target_angle_change >= 0.0 ){
        speed = v_l;
        ang_step_size_local = angular_step_size;
    }
    else {
        speed = -1.0 * v_l;
        ang_step_size_local = -1.0 * angular_step_size;
    }

    while ( std::abs( theta_change )  < std::abs( target_angle_change ) ){
        float dx = speed * cos( robot_yaw + ang_step_size_local/2);
        float dy = speed * sin( robot_yaw + ang_step_size_local/2);
        robotx = robotx + dx;
        roboty = roboty + dy;
        theta_change += ang_step_size_local;
        robot_yaw += ang_step_size_local ;
        posePerSample pose(robotx, roboty, robot_yaw);
        all_path_result[path_type].path_steps.push_back(pose);
    }
    init_yaw += target_angle_change;
    robot_yaw = mod2pi( init_yaw );
}


void ReedsSheppClass::get_samples_R( const float target_angle_change, float& robot_yaw, float& robotx, float& roboty, const std::string path_type  )
{
    if(target_angle_change > 0){
        float theta_change = 0;
        while (theta_change < target_angle_change)
            {
                float dx = v_l * cos( robot_yaw + angular_step_size/2);
                float dy = v_l * sin( robot_yaw + angular_step_size/2);
                robotx = robotx + (dx);
                roboty = roboty + (dy);
                theta_change += angular_step_size;
                robot_yaw -= angular_step_size ;

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
    if(target_angle_change < 0){
        float theta_change = 0;
        while (theta_change > target_angle_change)
            {
                float dx = v_l * cos( robot_yaw - angular_step_size/2);
                float dy = v_l * sin( robot_yaw - angular_step_size/2);
                robotx = robotx - (dx);
                roboty = roboty - (dy);
                theta_change -= angular_step_size;
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
}



void ReedsSheppClass::get_samples_S( const float target_angle_change, float robot_yaw, float& robotx, float& roboty, const std::string path_type  )
{
    float theta_change = 0;
    double linear_step_size_local = 0.0;
    if( target_angle_change >= 0.0 ){
        linear_step_size_local = linear_step_size;
    }
    else {
        linear_step_size_local = -1.0 * linear_step_size;
    }

    while ( std::abs( theta_change )  < std::abs( target_angle_change*turning_raius ) ){
        float dx = linear_step_size_local * cos( robot_yaw );
        float dy = linear_step_size_local * sin( robot_yaw );
        robotx = robotx + dx;
        roboty = roboty + dy;
        theta_change += linear_step_size_local;
        posePerSample pose(robotx, roboty, robot_yaw);
        all_path_result[path_type].path_steps.push_back(pose);
    }
}







