
#ifndef REEDSSHEPP_H
#define REEDSSHEPP_H

#include <iostream>
#include <string>
#include <queue>
#include <unordered_map>
#include <map>
#include <opencv2/core.hpp>
#include <algorithm>
#include <set>
#include <stdexcept>
#include <limits>
#include <math.h>

#include "reedshepp_path.h"

class ReedsSheppClass
{
private:

    std::array<double, 3> start_pose_;  // meters, rad
    std::array<double, 3> goal_pose_ ;  // meters, rad
    std::array<double, 3> goal_pose_processed_ ;  // rotated and unitless 

    double turning_raius_ = 0.5;  // meters
    
    double angular_step_size = 0.02;
    double linear_step_size = 0.05;

    double v_l ;

    void polar(double x, double y, double &radius_ro, double &theta);

    double mod2pi(double angle_radian);
    const double pi2_ = M_PI*2.0;  

    double rectify_angle_rad(double ang);


    const int obstacle_scan_range = 15;
    const int map_inflation_size = 5;

    void map_inflation(cv::Mat* mapIn, cv::Mat* mapOut, int infla_size);

    void sample_points_on_path();

    void LpSpLp_base( double x, double y, double phi,  double &t, double &u, double &v, double& L, bool& valid);

    void LpSpRp_base( double x, double y, double phi,  double &t, double &u, double &v, double& L, bool& valid);

    void LpRmL_base(double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid);

    void LpSpLp(   );
    void LpSpRp(   );
    void LmSmLm(   );
    void LmSmRm(   );
    void RpSpRp(   );
    void RpSpLp(   );
    void RmSmRm(   );
    void RmSmLm(   );

    void LpRmL (   );
    void LmRpLm(   );
    void RpLmRp(   );
    void RmLpRm(   );
    void LpRmLm(   );
    void LmRpLp(   );
    void RpLmRm(   );
    void RmLpRp(   );

    void get_samples_L( const double target_angle_change, double& robot_yaw, double& robotx, double& roboty, ClassReedSheppPath::pathResult& result_container); // const std::string path_type  );
    void get_samples_R( const double target_angle_change, double& robot_yaw, double& robotx, double& roboty, ClassReedSheppPath::pathResult& result_container); // const std::string path_type  );
    void get_samples_S( const double target_angle_change, double  robot_yaw, double& robotx, double& roboty, ClassReedSheppPath::pathResult& result_container); // const std::string path_type  );

public:
    ReedsSheppClass();

    ~ReedsSheppClass();

    ClassReedSheppPath::PathCollection all_possible_paths_;

    void setup( std::array<double, 3> start_pose, std::array<double, 3> goal_pose );

    // std::deque< std::array<int,3>> path;

    void search( );

    void get_path( std::array<double,3> start_pose, std::array<double,3> goal_pose , std::string path_type, std::vector<double>& path_angle, bool& valid);

};


void ReedsSheppClass::setup(  std::array<double, 3> start_pose, std::array<double, 3> goal_pose )
{
    start_pose_ = start_pose;
    goal_pose_ = goal_pose;

    v_l = 2*turning_raius_*sin(angular_step_size/2) ;

    double dy = goal_pose_[1] - start_pose_[1];
    double dx = goal_pose_[0] - start_pose_[0];
    double cos_theta1 = cos(start_pose_[2]);
    double sin_theta1 = sin(start_pose_[2]);

    // std::cout << "goal pose: " << goal_pose[0] << " " << goal_pose[1]  << " " << goal_pose[2] << std::endl;  

    double goal_x_rotated_unitless = ( dx * cos_theta1 + dy * sin_theta1) / turning_raius_;
    double goal_y_rotated_unitless = (-dx * sin_theta1 + dy * cos_theta1) / turning_raius_;
    double goal_theta_rotated = rectify_angle_rad( goal_pose[2] - start_pose[2] );

    goal_pose_processed_ = { goal_x_rotated_unitless, goal_y_rotated_unitless, goal_theta_rotated };

    all_possible_paths_.reset(); 

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

double ReedsSheppClass::rectify_angle_rad(double ang)
{
    while (ang > 2 * M_PI)
        ang -= 2 * M_PI;
    while (ang < 0)
        ang += 2 * M_PI;
    return ang;
}




void ReedsSheppClass::search(  )
{
    std::vector<double> path_angle;
    bool valid;

    LpSpLp();
    LmSmLm();
    RpSpRp();
    RmSmRm();

    LpSpRp();
    LmSmRm();
    RpSpLp();
    RmSmLm();

    LpRmL();
    LmRpLm();
    RpLmRp();
    RmLpRm();

    LpRmLm();
    LmRpLp( );
    RpLmRm( );
    RmLpRp( );


}

///////////////////////////   CSC 
// 8.1 in paper 
void ReedsSheppClass::LpSpLp_base( double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid )
{
    valid = false;

    polar( x-sin(phi), y-1+cos(phi), u, t );
    v = mod2pi(phi - t);
    L = std::abs(t) + std::abs(u) + std::abs(v) ;
    if( t>0.0 && v > 0.0){
        valid = true;
    }
    else{
        valid = false;
    }
    // valid = true;
    // std::cout << "phi :" << phi << std::endl;
    // std::cout << "t :" << t << std::endl;
    // std::cout << "v :" << v << std::endl;
}

void ReedsSheppClass::LpSpLp( ){
    double x   = goal_pose_processed_[0];
    double y   = goal_pose_processed_[1];
    double phi = goal_pose_processed_[2];
    double t, u, v, L; bool valid;
    LpSpLp_base( x, y, phi, t, u, v, L, valid );
    all_possible_paths_.LpSpLp.valid = valid;
    all_possible_paths_.LpSpLp.path_word = "LpSpLp";
    if( valid ){
        all_possible_paths_.LpSpLp.path_length_unitless = L; 
        double robot_x     = start_pose_[0] ;
        double robot_y     = start_pose_[1] ;
        double robot_theta = start_pose_[2] ;
        // sample points on path. update path in pathcollection. 
        get_samples_L( t, robot_theta, robot_x, robot_y, all_possible_paths_.LpSpLp);
        get_samples_S( u, robot_theta, robot_x, robot_y, all_possible_paths_.LpSpLp);
        get_samples_L( v, robot_theta, robot_x, robot_y, all_possible_paths_.LpSpLp);
    }
}


void ReedsSheppClass::LmSmLm(){
    double x   = goal_pose_processed_[0];
    double y   = goal_pose_processed_[1];
    double phi = goal_pose_processed_[2];
    double t, u, v, L; bool valid;
    LpSpLp_base(-x,y,-phi,t,u,v,L,valid);
    all_possible_paths_.LmSmLm.path_word = "LmSmLm";
    all_possible_paths_.LmSmLm.valid = valid;
    if( valid ){
        all_possible_paths_.LmSmLm.path_length_unitless = L; 
        double robot_x     = start_pose_[0] ;
        double robot_y     = start_pose_[1] ;
        double robot_theta = start_pose_[2] ;
        get_samples_L( -t, robot_theta, robot_x, robot_y, all_possible_paths_.LmSmLm);
        get_samples_S( -u, robot_theta, robot_x, robot_y, all_possible_paths_.LmSmLm);
        get_samples_L( -v, robot_theta, robot_x, robot_y, all_possible_paths_.LmSmLm);
    }
}



void ReedsSheppClass::RpSpRp( ){
    double x   = goal_pose_processed_[0];
    double y   = goal_pose_processed_[1];
    double phi = goal_pose_processed_[2];
    double t, u, v, L; bool valid;
    LpSpLp_base(x,-y,-phi,t,u,v,L,valid);
    all_possible_paths_.RpSpRp.path_word = "RpSpRp";
    all_possible_paths_.RpSpRp.valid = valid;
    if( valid ){
        all_possible_paths_.RpSpRp.path_length_unitless = L; 
        double robot_x     = start_pose_[0] ;
        double robot_y     = start_pose_[1] ;
        double robot_theta = start_pose_[2] ;
        get_samples_R( t, robot_theta, robot_x, robot_y, all_possible_paths_.RpSpRp);
        get_samples_S( u, robot_theta, robot_x, robot_y, all_possible_paths_.RpSpRp);
        get_samples_R( v, robot_theta, robot_x, robot_y, all_possible_paths_.RpSpRp);
    }
}


void ReedsSheppClass::RmSmRm( ){
    double x   = goal_pose_processed_[0];
    double y   = goal_pose_processed_[1];
    double phi = goal_pose_processed_[2];
    double t, u, v, L; bool valid;
    LpSpLp_base(-x,-y, phi,t,u,v,L,valid);
    all_possible_paths_.RmSmRm.path_word = "RmSmRm";
    all_possible_paths_.RmSmRm.valid = valid;
    if( valid ){
        all_possible_paths_.RmSmRm.path_length_unitless = L; 
        double robot_x     = start_pose_[0] ;
        double robot_y     = start_pose_[1] ;
        double robot_theta = start_pose_[2] ;
        get_samples_R( -t, robot_theta, robot_x, robot_y, all_possible_paths_.RmSmRm);
        get_samples_S( -u, robot_theta, robot_x, robot_y, all_possible_paths_.RmSmRm);
        get_samples_R( -v, robot_theta, robot_x, robot_y, all_possible_paths_.RmSmRm);
    }
}


void ReedsSheppClass::LpSpRp_base( double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid)
{
    valid = false;
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

void ReedsSheppClass::LpSpRp( ){
    double x   = goal_pose_processed_[0];
    double y   = goal_pose_processed_[1];
    double phi = goal_pose_processed_[2];
    double t, u, v, L; bool valid;
    LpSpRp_base(-x,y,-phi,t,u,v,L,valid);
    all_possible_paths_.LpSpRp.path_word = "LpSpRp";
    all_possible_paths_.LpSpRp.valid = valid;
    if( valid ){
        all_possible_paths_.LpSpRp.path_length_unitless = L; 
        double robot_x     = start_pose_[0] ;
        double robot_y     = start_pose_[1] ;
        double robot_theta = start_pose_[2] ;
        get_samples_L( t, robot_theta, robot_x, robot_y, all_possible_paths_.LpSpRp);
        get_samples_S( u, robot_theta, robot_x, robot_y, all_possible_paths_.LpSpRp);
        get_samples_R( v, robot_theta, robot_x, robot_y, all_possible_paths_.LpSpRp);
    }
}

void ReedsSheppClass::LmSmRm( ){
    double x   = goal_pose_processed_[0];
    double y   = goal_pose_processed_[1];
    double phi = goal_pose_processed_[2];
    double t, u, v, L; bool valid;
    LpSpRp_base(-x,y,-phi,t,u,v,L,valid);
    all_possible_paths_.LmSmRm.path_word = "LmSmRm";
    all_possible_paths_.LmSmRm.valid = valid;
    if( valid ){
        all_possible_paths_.LmSmRm.path_length_unitless = L; 
        double robot_x     = start_pose_[0] ;
        double robot_y     = start_pose_[1] ;
        double robot_theta = start_pose_[2] ;
        get_samples_L( -t, robot_theta, robot_x, robot_y, all_possible_paths_.LmSmRm);
        get_samples_S( -u, robot_theta, robot_x, robot_y, all_possible_paths_.LmSmRm);
        get_samples_R( -v, robot_theta, robot_x, robot_y, all_possible_paths_.LmSmRm);
    }
}

void ReedsSheppClass::RpSpLp( ){
    double x   = goal_pose_processed_[0];
    double y   = goal_pose_processed_[1];
    double phi = goal_pose_processed_[2];
    double t, u, v, L; bool valid;
    LpSpRp_base(x, -y,-phi,t,u,v,L,valid);
    all_possible_paths_.RpSpLp.path_word = "RpSpLp";
    all_possible_paths_.RpSpLp.valid = valid;
    if( valid ){
        all_possible_paths_.RpSpLp.path_length_unitless = L; 
        double robot_x     = start_pose_[0] ;
        double robot_y     = start_pose_[1] ;
        double robot_theta = start_pose_[2] ;
        get_samples_R( t, robot_theta, robot_x, robot_y, all_possible_paths_.RpSpLp);
        get_samples_S( u, robot_theta, robot_x, robot_y, all_possible_paths_.RpSpLp);
        get_samples_L( v, robot_theta, robot_x, robot_y, all_possible_paths_.RpSpLp);
    }
}

void ReedsSheppClass::RmSmLm( ){
    double x   = goal_pose_processed_[0];
    double y   = goal_pose_processed_[1];
    double phi = goal_pose_processed_[2];
    double t, u, v, L; bool valid;
    LpSpRp_base(-x,-y, phi,t,u,v,L,valid);
    all_possible_paths_.RmSmLm.path_word = "RmSmLm";
    all_possible_paths_.RmSmLm.valid = valid;
    if( valid ){
        all_possible_paths_.RmSmLm.path_length_unitless = L; 
        double robot_x     = start_pose_[0] ;
        double robot_y     = start_pose_[1] ;
        double robot_theta = start_pose_[2] ;
        get_samples_R( -t, robot_theta, robot_x, robot_y, all_possible_paths_.RmSmLm);
        get_samples_S( -u, robot_theta, robot_x, robot_y, all_possible_paths_.RmSmLm);
        get_samples_L( -v, robot_theta, robot_x, robot_y, all_possible_paths_.RmSmLm);
    }
}


void ReedsSheppClass::LpRmL_base(double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid){
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
    L = std::abs(t) + std::abs(u) + std::abs(v) ;
}


void ReedsSheppClass::LpRmL( ){
    double x   = goal_pose_processed_[0];
    double y   = goal_pose_processed_[1];
    double phi = goal_pose_processed_[2];
    double t, u, v, L; bool valid;
    LpRmL_base(x,y, phi,t,u,v,L,valid);
    all_possible_paths_.LpRmL.path_word = "LpRmL";
    all_possible_paths_.LpRmL.valid = valid;
    if( valid ){
        all_possible_paths_.LpRmL.path_length_unitless = L; 
        double robot_x     = start_pose_[0] ;
        double robot_y     = start_pose_[1] ;
        double robot_theta = start_pose_[2] ;
        get_samples_L( t, robot_theta, robot_x, robot_y, all_possible_paths_.LpRmL);
        get_samples_R( u, robot_theta, robot_x, robot_y, all_possible_paths_.LpRmL);
        get_samples_L( v, robot_theta, robot_x, robot_y, all_possible_paths_.LpRmL);
    }
    // std::cout << "LpRmL  " << valid  << std::endl;
}

void ReedsSheppClass::LmRpLm( ){
    double x   = goal_pose_processed_[0];
    double y   = goal_pose_processed_[1];
    double phi = goal_pose_processed_[2];
    double t, u, v, L; bool valid;
    LpRmL_base(-x,y, -phi,t,u,v,L,valid);
    all_possible_paths_.LmRpLm.path_word = "LmRpLm";
    all_possible_paths_.LmRpLm.valid = valid;
    if( valid ){
        all_possible_paths_.LmRpLm.path_length_unitless = L; 
        double robot_x     = start_pose_[0] ;
        double robot_y     = start_pose_[1] ;
        double robot_theta = start_pose_[2] ;
        get_samples_L( -t, robot_theta, robot_x, robot_y, all_possible_paths_.LmRpLm);
        get_samples_R( -u, robot_theta, robot_x, robot_y, all_possible_paths_.LmRpLm);
        get_samples_L( -v, robot_theta, robot_x, robot_y, all_possible_paths_.LmRpLm);
    }
    // std::cout << "LmRpLm  " << valid  << std::endl;
}


void ReedsSheppClass::RpLmRp( ){
    double x   = goal_pose_processed_[0];
    double y   = goal_pose_processed_[1];
    double phi = goal_pose_processed_[2];
    double t, u, v, L; bool valid;
    LpRmL_base( x, -y, -phi,t,u,v,L,valid);
    all_possible_paths_.RpLmRp.path_word = "RpLmRp";
    all_possible_paths_.RpLmRp.valid = valid;
    if( valid ){
        all_possible_paths_.RpLmRp.path_length_unitless = L; 
        double robot_x     = start_pose_[0] ;
        double robot_y     = start_pose_[1] ;
        double robot_theta = start_pose_[2] ;
        get_samples_R( t, robot_theta, robot_x, robot_y, all_possible_paths_.RpLmRp);
        get_samples_L( u, robot_theta, robot_x, robot_y, all_possible_paths_.RpLmRp);
        get_samples_R( v, robot_theta, robot_x, robot_y, all_possible_paths_.RpLmRp);
    }
    // std::cout << "RpLmRp  " << valid  << std::endl;
}

void ReedsSheppClass::RmLpRm( ){
    double x   = goal_pose_processed_[0];
    double y   = goal_pose_processed_[1];
    double phi = goal_pose_processed_[2];
    double t, u, v, L; bool valid;
    LpRmL_base( -x, -y, phi,t,u,v,L,valid);
    all_possible_paths_.RmLpRm.path_word = "RmLpRm";
    all_possible_paths_.RmLpRm.valid = valid;
    if( valid ){
        all_possible_paths_.RmLpRm.path_length_unitless = L; 
        double robot_x     = start_pose_[0] ;
        double robot_y     = start_pose_[1] ;
        double robot_theta = start_pose_[2] ;
        get_samples_R( -t, robot_theta, robot_x, robot_y, all_possible_paths_.RmLpRm);
        get_samples_L( -u, robot_theta, robot_x, robot_y, all_possible_paths_.RmLpRm);
        get_samples_R( -v, robot_theta, robot_x, robot_y, all_possible_paths_.RmLpRm);
    }
    // std::cout << "RmLpRm  " << valid  << std::endl;
}


void ReedsSheppClass::LpRmLm( ){
    double x   = goal_pose_processed_[0];
    double y   = goal_pose_processed_[1];
    double phi = goal_pose_processed_[2];
    double t, u, v, L; bool valid;
    double xb = x*cos(phi)+y*sin(phi);
    double yb = x*sin(phi)-y*cos(phi);
    LpRmL_base( xb, yb, phi, t, u, v, L, valid);
    all_possible_paths_.LpRmLm.path_word = "LpRmLm";
    all_possible_paths_.LpRmLm.valid = valid;
    if( valid ){
        v = M_PI*2.0 - v;
        L = std::abs(t) + std::abs(u) + std::abs(v) ; 
        all_possible_paths_.LpRmLm.path_length_unitless = L; 
        double robot_x     = start_pose_[0] ;
        double robot_y     = start_pose_[1] ;
        double robot_theta = start_pose_[2] ;
        get_samples_L( t, robot_theta, robot_x, robot_y, all_possible_paths_.LpRmLm);
        get_samples_R( u, robot_theta, robot_x, robot_y, all_possible_paths_.LpRmLm);
        get_samples_L( -v, robot_theta, robot_x, robot_y, all_possible_paths_.LpRmLm);
    }
    // std::cout << "LpRmLm  " << valid  << " " << t << " " << u << " " << v << std::endl;
}


void ReedsSheppClass::LmRpLp( ){
    double x   = goal_pose_processed_[0];
    double y   = goal_pose_processed_[1];
    double phi = goal_pose_processed_[2];
    double t, u, v, L; bool valid;
    double xb = x*cos(phi)+y*sin(phi);
    double yb = x*sin(phi)-y*cos(phi);
    LpRmL_base( -xb, yb, -phi, t, u, v, L, valid);
    all_possible_paths_.LmRpLp.path_word = "LmRpLp";
    all_possible_paths_.LmRpLp.valid = valid;
    if( valid ){
        v = M_PI*2.0 - v;
        L = std::abs(t) + std::abs(u) + std::abs(v) ; 
        all_possible_paths_.LmRpLp.path_length_unitless = L; 
        double robot_x     = start_pose_[0] ;
        double robot_y     = start_pose_[1] ;
        double robot_theta = start_pose_[2] ;
        get_samples_L( -t, robot_theta, robot_x, robot_y, all_possible_paths_.LmRpLp);
        get_samples_R( -u, robot_theta, robot_x, robot_y, all_possible_paths_.LmRpLp);
        get_samples_L( v, robot_theta, robot_x, robot_y, all_possible_paths_.LmRpLp);
    }
    // std::cout << "LmRpLp  " << valid  << " " << t << " " << u << " " << v << std::endl;
}

void ReedsSheppClass::RpLmRm( ){
    double x   = goal_pose_processed_[0];
    double y   = goal_pose_processed_[1];
    double phi = goal_pose_processed_[2];
    double t, u, v, L; bool valid;
    double xb = x*cos(phi)+y*sin(phi);
    double yb = x*sin(phi)-y*cos(phi);
    LpRmL_base( xb, -yb, -phi, t, u, v, L, valid);
    all_possible_paths_.RpLmRm.path_word = "RpLmRm";
    all_possible_paths_.RpLmRm.valid = valid;
    if( valid ){
        v = M_PI*2.0 - v;
        L = std::abs(t) + std::abs(u) + std::abs(v) ; 
        all_possible_paths_.RpLmRm.path_length_unitless = L; 
        double robot_x     = start_pose_[0] ;
        double robot_y     = start_pose_[1] ;
        double robot_theta = start_pose_[2] ;
        get_samples_R( t, robot_theta, robot_x, robot_y, all_possible_paths_.RpLmRm);
        get_samples_L( u, robot_theta, robot_x, robot_y, all_possible_paths_.RpLmRm);
        get_samples_R( -v, robot_theta, robot_x, robot_y, all_possible_paths_.RpLmRm);
    }
    std::cout << "RpLmRm  " << valid  << " " << t << " " << u << " " << v << std::endl;
}


void ReedsSheppClass::RmLpRp( ){
    double x   = goal_pose_processed_[0];
    double y   = goal_pose_processed_[1];
    double phi = goal_pose_processed_[2];
    double t, u, v, L; bool valid;
    double xb = x*cos(phi)+y*sin(phi);
    double yb = x*sin(phi)-y*cos(phi);
    LpRmL_base( -xb, -yb, phi, t, u, v, L, valid);
    all_possible_paths_.RmLpRp.path_word = "RmLpRp";
    all_possible_paths_.RmLpRp.valid = valid;
    if( valid ){
        v = M_PI*2.0 - v;
        L = std::abs(t) + std::abs(u) + std::abs(v) ; 
        all_possible_paths_.RmLpRp.path_length_unitless = L; 
        double robot_x     = start_pose_[0] ;
        double robot_y     = start_pose_[1] ;
        double robot_theta = start_pose_[2] ;
        get_samples_R( -t, robot_theta, robot_x, robot_y, all_possible_paths_.RmLpRp);
        get_samples_L( -u, robot_theta, robot_x, robot_y, all_possible_paths_.RmLpRp);
        get_samples_R( v, robot_theta, robot_x, robot_y, all_possible_paths_.RmLpRp);
    }
    // std::cout << "RmLpRp  " << valid  << " " << t << " " << u << " " << v << std::endl;
}



































void ReedsSheppClass::get_samples_L( const double target_angle_change, double& robot_yaw, double& robotx, double& roboty, ClassReedSheppPath::pathResult& result_container )// const std::string path_type  )
{
    double init_yaw = robot_yaw;
    double theta_change = 0;
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
        double dx = speed * cos( robot_yaw + ang_step_size_local/2);
        double dy = speed * sin( robot_yaw + ang_step_size_local/2);
        robotx = robotx + dx;
        roboty = roboty + dy;
        theta_change += ang_step_size_local;
        robot_yaw += ang_step_size_local ;
        ClassReedSheppPath::posePerSample pose(robotx, roboty, robot_yaw);
        result_container.path_steps.push_back(pose);
    }
    init_yaw += target_angle_change;
    robot_yaw = mod2pi( init_yaw );
}


void ReedsSheppClass::get_samples_R( const double target_angle_change, double& robot_yaw, double& robotx, double& roboty, ClassReedSheppPath::pathResult& result_container )//, const std::string path_type  )
{
    double init_yaw = robot_yaw;
    double theta_change = 0;
    double speed = 0.0;
    double ang_step_size_local = 0.0;
    if( target_angle_change >= 0.0 ){
        speed = v_l;
        ang_step_size_local = -1.0 * angular_step_size;
    }
    else {
        speed = -1.0 * v_l;
        ang_step_size_local = angular_step_size;
    }
    while ( std::abs( theta_change )  < std::abs( target_angle_change ) ){
        double dx = speed * cos( robot_yaw + ang_step_size_local/2);
        double dy = speed * sin( robot_yaw + ang_step_size_local/2);
        robotx = robotx + dx;
        roboty = roboty + dy;
        theta_change += ang_step_size_local;
        robot_yaw += ang_step_size_local ;
        ClassReedSheppPath::posePerSample pose(robotx, roboty, robot_yaw);
        // all_path_result[path_type].path_steps.push_back(pose);
        result_container.path_steps.push_back(pose);
    }
    init_yaw -= target_angle_change;
    robot_yaw = mod2pi( init_yaw );


    // if(target_angle_change > 0){
    //     double theta_change = 0;
    //     while (theta_change < target_angle_change)
    //         {
    //             double dx = v_l * cos( robot_yaw + angular_step_size/2);
    //             double dy = v_l * sin( robot_yaw + angular_step_size/2);
    //             robotx = robotx + (dx);
    //             roboty = roboty + (dy);
    //             theta_change += angular_step_size;
    //             robot_yaw -= angular_step_size ;

    //             ClassReedSheppPath::posePerSample pose(robotx, roboty, robot_yaw);
    //             // all_path_result[path_type].path_steps.push_back(pose);
    //             result_container.path_steps.push_back(pose);

    //             // std::cout << " raius " << turning_raius ;
    //             // std::cout << "  robot_theta "  << int(robot_theta*180.0/M_PI) ;
    //             // std::cout << "  x " << robot_x << " y " << robot_y ;
    //             // std::cout << "  angular_step_size " << angular_step_size  ;
    //             // std::cout << "  theta_change " << theta_change  ;
    //             // std::cout << "  dx " << dx << " dy " << dy << std::endl;
    //             // std::cout << robot_theta*180.0/M_PI << " " << robot_x << " " << robot_y << " " << v_l  << " " << dx << " " << dy << std::endl;
    //         }
    // }
    // if(target_angle_change < 0){
    //     double theta_change = 0;
    //     while (theta_change > target_angle_change)
    //         {
    //             double dx = v_l * cos( robot_yaw - angular_step_size/2);
    //             double dy = v_l * sin( robot_yaw - angular_step_size/2);
    //             robotx = robotx - (dx);
    //             roboty = roboty - (dy);
    //             theta_change -= angular_step_size;
    //             robot_yaw += angular_step_size ;

    //             // cv::circle(map_for_view, cv::Point(robotx, roboty), 1, cv::Scalar(0,255,0),  -1);

    //             ClassReedSheppPath::posePerSample pose(robotx, roboty, robot_yaw);
    //             // all_path_result[path_type].path_steps.push_back(pose);
    //             result_container.path_steps.push_back(pose);

    //             // std::cout << " raius " << turning_raius ;
    //             // std::cout << "  robot_theta "  << int(robot_theta*180.0/M_PI) ;
    //             // std::cout << "  x " << robot_x << " y " << robot_y ;
    //             // std::cout << "  angular_step_size " << angular_step_size  ;
    //             // std::cout << "  theta_change " << theta_change  ;
    //             // std::cout << "  dx " << dx << " dy " << dy << std::endl;
    //             // std::cout << robot_theta*180.0/M_PI << " " << robot_x << " " << robot_y << " " << v_l  << " " << dx << " " << dy << std::endl;
    //         }
    // }
}



void ReedsSheppClass::get_samples_S( const double target_angle_change, double  robot_yaw, double& robotx, double& roboty, ClassReedSheppPath::pathResult& result_container ) 
{
    double init_x = robotx;
    double init_y = roboty;
    double theta_change = 0;
    double linear_step_size_local = 0.0;
    if( target_angle_change >= 0.0 ){
        linear_step_size_local = linear_step_size;
    }
    else {
        linear_step_size_local = -1.0 * linear_step_size;
    }

    while ( std::abs( theta_change )  < std::abs( target_angle_change*turning_raius_ ) ){
        double dx = linear_step_size_local * cos( robot_yaw );
        double dy = linear_step_size_local * sin( robot_yaw );
        robotx = robotx + dx;
        roboty = roboty + dy;
        theta_change += linear_step_size_local;
        ClassReedSheppPath::posePerSample pose(robotx, roboty, robot_yaw);
        // all_path_result[path_type].path_steps.push_back(pose);
        result_container.path_steps.push_back(pose);
    }
    robotx = init_x + target_angle_change*turning_raius_ * cos( robot_yaw );
    roboty = init_y + target_angle_change*turning_raius_ * sin( robot_yaw );
}




ReedsSheppClass::ReedsSheppClass(  )
{
}

ReedsSheppClass::~ReedsSheppClass()
{
}



#endif