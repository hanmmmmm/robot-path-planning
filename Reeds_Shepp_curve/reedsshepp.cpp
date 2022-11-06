
#include <iostream>

#include <string>
#include <stdexcept>

#include <limits>
#include <math.h>

#include "utils/math_tools.h"


// void ReedsSheppClass::LpRmL(double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid){
//     double xi = x - sin(phi); 
//     double eta = y - 1. + cos(phi); 
//     double u1, theta;
//     polar(xi, eta, u1, theta);
    
//     if (u1 > 4.0){
//         valid = false;
//     }
//     else{
//         u = -2.0 * asin( u1 / 4.0 );
//         t = mod2pi(theta + 0.5 * u + M_PI);
//         v = mod2pi(phi - t + u);
//         if (t >= 0.0 && u <= 0.0){
//             valid = true;
//         }
//         else{
//             valid = false;
//         }
//     }
// }


    

// void ReedsSheppClass::LmRpLm(double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid){
//     LpRmL(-x, y, -phi, t, u, v, L, valid);
// }

// void ReedsSheppClass::RpLmRp(double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid){
//     LpRmL(x, -y, -phi, t, u, v, L, valid);
// }

// void ReedsSheppClass::RmLpRm(double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid){
//     LpRmL(-x, -y, phi, t, u, v, L, valid);
// }

// // void ReedsSheppClass::LpRmLm(double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid){
// //     double xb = x*cos(phi)+y*sin(phi);
// //     double yb = x*sin(phi)-y*cos(phi);
// //     LpRmL( xb, yb, phi, t, u, v, L, valid);
// // }

// // void ReedsSheppClass::LmRpLp(double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid){
// //     double xb = x*cos(phi)+y*sin(phi);
// //     double yb = x*sin(phi)-y*cos(phi);
// //     LpRmL( -xb, yb, -phi, t, u, v, L, valid);
// // }

// // void ReedsSheppClass::RpLmRm(double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid){
// //     double xb = x*cos(phi)+y*sin(phi);
// //     double yb = x*sin(phi)-y*cos(phi);
// //     LpRmL( xb, -yb, -phi, t, u, v, L, valid);
// // }


// // void ReedsSheppClass::RmLpRp(double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid){
// //     double xb = x*cos(phi)+y*sin(phi);
// //     double yb = x*sin(phi)-y*cos(phi);
// //     LpRmL( -xb, -yb, phi, t, u, v, L, valid);
// // }


// void ReedsSheppClass::get_path( std::array<double,3> start_pose, std::array<double,3> goal_pose , std::string path_type, std::vector<double>& path_angle, bool& valid)
// {
//     double dy = goal_pose[1] - start_pose[1];
//     double dx = goal_pose[0] - start_pose[0];
//     double cos_theta1 = cos(start_pose[2]);
//     double sin_theta1 = sin(start_pose[2]);

//     // std::cout << "goal pose: " << goal_pose[0] << " " << goal_pose[1]  << " " << goal_pose[2] << std::endl;  

//     double goal_x_rotated = (dx * cos_theta1 + dy * sin_theta1) / turning_raius;
//     double goal_y_rotated = (-dx * sin_theta1 + dy * cos_theta1) / turning_raius;
//     double goal_theta = rectify_angle_rad( goal_pose[2] - start_pose[2] );

//     double robot_x     = start_pose[0] ;
//     double robot_y     = start_pose[1] ;
//     double robot_theta = start_pose[2] ;

//     double theta_change = 0;


//     if(path_type == "LpSpLp" ){
//         double t,u,v, L; 
//         LpSpLp(goal_x_rotated, goal_y_rotated,goal_theta, t,u,v,L,valid);
//         if( valid ){
//             std::cout << "t " << t << std::endl;
//             std::cout << "u " << u << std::endl;
//             std::cout << "v " << v << std::endl;

//             get_samples_L( t, robot_theta, robot_x, robot_y, path_type);
//             robot_theta = start_pose[2] + t;
//             get_samples_S( u, robot_theta, robot_x, robot_y, path_type);

//             get_samples_L( v, robot_theta, robot_x, robot_y, path_type);
//         }
//     }
//     if(path_type == "LpSpRp" ){
//         double t,u,v, L; 
//         LpSpRp(goal_x_rotated, goal_y_rotated,goal_theta, t,u,v,L,valid);
//         if( valid ){
//             std::cout << "t " << t << std::endl;
//             std::cout << "u " << u << std::endl;
//             std::cout << "v " << v << std::endl;

//             get_samples_L( t, robot_theta, robot_x, robot_y, path_type);
//             robot_theta = start_pose[2] + t;
//             get_samples_S( u, robot_theta, robot_x, robot_y, path_type);
//             get_samples_R( v, robot_theta, robot_x, robot_y, path_type);
//         }
//         else{
//             std::cout << "           Invalid pose for LpSpRp" << std::endl;
//         }
//     }
//     if(path_type == "LmSmLm" ){
//         double t,u,v, L; 
//         LmSmLm(goal_x_rotated, goal_y_rotated,goal_theta, t,u,v,L,valid);
//         if( valid ){
//             std::cout << "t " << t << std::endl;
//             std::cout << "u " << u << std::endl;
//             std::cout << "v " << v << std::endl;

//             get_samples_L( -t, robot_theta, robot_x, robot_y, path_type);
//             robot_theta = start_pose[2] - t;
//             get_samples_S( -u, robot_theta, robot_x, robot_y, path_type);

//             get_samples_L( -v, robot_theta, robot_x, robot_y, path_type);
//         }
//     }

//     if(path_type == "LmSmRm" ){
//         double t,u,v, L; 
//         LmSmRm(goal_x_rotated, goal_y_rotated,goal_theta, t,u,v,L,valid);
//         if( valid ){
//             std::cout << "t " << t << std::endl;
//             std::cout << "u " << u << std::endl;
//             std::cout << "v " << v << std::endl;

//             get_samples_L( -t, robot_theta, robot_x, robot_y, path_type);
//             robot_theta = start_pose[2] - t;
//             get_samples_S( -u, robot_theta, robot_x, robot_y, path_type);
//             get_samples_R( -v, robot_theta, robot_x, robot_y, path_type);
//         }
//     }

//     if(path_type == "RpSpRp" ){
//         double t,u,v, L; 
//         RpSpRp(goal_x_rotated, goal_y_rotated,goal_theta, t,u,v,L,valid);
//         if( valid ){
//             std::cout << "t " << t << std::endl;
//             std::cout << "u " << u << std::endl;
//             std::cout << "v " << v << std::endl;

//             get_samples_R( t, robot_theta, robot_x, robot_y, path_type);
//             robot_theta = start_pose[2] - t;
//             get_samples_S( u, robot_theta, robot_x, robot_y, path_type);
//             get_samples_R( v, robot_theta, robot_x, robot_y, path_type);
//         }
//     }

//     if(path_type == "RpSpLp" ){
//         double t,u,v, L; 
//         RpSpLp(goal_x_rotated, goal_y_rotated,goal_theta, t,u,v,L,valid);
//         if( valid ){
//             std::cout << "t " << t << std::endl;
//             std::cout << "u " << u << std::endl;
//             std::cout << "v " << v << std::endl;

//             get_samples_R( t, robot_theta, robot_x, robot_y, path_type);
//             robot_theta = start_pose[2] - t;
//             get_samples_S( u, robot_theta, robot_x, robot_y, path_type);
//             get_samples_L( v, robot_theta, robot_x, robot_y, path_type);
//         }
//     }

//     if(path_type == "RmSmRm" ){
//         double t,u,v, L; 
//         RmSmRm(goal_x_rotated, goal_y_rotated,goal_theta, t,u,v,L,valid);
//         if( valid ){
//             std::cout << "t " << t << std::endl;
//             std::cout << "u " << u << std::endl;
//             std::cout << "v " << v << std::endl;

//             get_samples_R( -t, robot_theta, robot_x, robot_y, path_type);
//             robot_theta = start_pose[2] + t;
//             get_samples_S( -u, robot_theta, robot_x, robot_y, path_type);
//             get_samples_R( -v, robot_theta, robot_x, robot_y, path_type);
//         }
//     }

//     if(path_type == "RmSmLm" ){
//         double t,u,v, L; 
//         RmSmLm(goal_x_rotated, goal_y_rotated,goal_theta, t,u,v,L,valid);
//         if( valid ){
//             std::cout << "t " << t << std::endl;
//             std::cout << "u " << u << std::endl;
//             std::cout << "v " << v << std::endl;

//             get_samples_R( -t, robot_theta, robot_x, robot_y, path_type);
//             robot_theta = start_pose[2] + t;
//             get_samples_S( -u, robot_theta, robot_x, robot_y, path_type);
//             get_samples_L( -v, robot_theta, robot_x, robot_y, path_type);
//         }
//     }

//     if(path_type == "LpRmL" ){
//         double t,u,v, L; 
//         LpRmL(goal_x_rotated, goal_y_rotated,goal_theta, t,u,v,L,valid);
//         if( valid ){
//             std::cout << "t " << t << std::endl;
//             std::cout << "u " << u << std::endl;
//             std::cout << "v " << v << std::endl;

//             get_samples_L( t, robot_theta, robot_x, robot_y, path_type);
//             // robot_theta = start_pose[2] + t;
//             std::cout << "robot_theta " << robot_theta << std::endl;
//             get_samples_R( u, robot_theta, robot_x, robot_y, path_type);
//             // robot_theta += u;
//             std::cout << "robot_theta " << robot_theta << std::endl;
//             get_samples_L( v, robot_theta, robot_x, robot_y, path_type);
//         }
//     }

//     if(path_type == "RpLmRp" ){
//         double t,u,v, L; 
//         RpLmRp(goal_x_rotated, goal_y_rotated,goal_theta, t,u,v,L,valid);
//         if( valid ){
//             std::cout << "t " << t << std::endl;
//             std::cout << "u " << u << std::endl;
//             std::cout << "v " << v << std::endl;
//             get_samples_R( t, robot_theta, robot_x, robot_y, path_type);
//             // robot_theta = start_pose[2] + t;
//             std::cout << "robot_theta " << robot_theta << std::endl;
//             get_samples_L( u, robot_theta, robot_x, robot_y, path_type);
//             // robot_theta += u;
//             std::cout << "robot_theta " << robot_theta << std::endl;
//             get_samples_R( v, robot_theta, robot_x, robot_y, path_type);
//         }
//     }
// }





