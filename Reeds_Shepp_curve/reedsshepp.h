
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

    double ccc_d_ ;
    double ccc_alpha_ ;
    double ccc_beta_ ;

    void polar(double x, double y, double &radius_ro, double &theta);

    double mod2pi(double angle_radian);
    const double pi2_ = M_PI*2.0;  

    double rectify_angle_rad(double ang);

    void tauOmega(double u, double v, double xi, double eta, double phi, double &tau, double &omega);


    const int obstacle_scan_range = 15;
    const int map_inflation_size = 5;

    void map_inflation(cv::Mat* mapIn, cv::Mat* mapOut, int infla_size);

    void sample_points_on_path();

    void LpSpLp_base( double x, double y, double phi,  double &t, double &u, double &v, double& L, bool& valid);

    void LpSpRp_base( double x, double y, double phi,  double &t, double &u, double &v, double& L, bool& valid);

    // void LpRmL_base(double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid);
    // void LpRmLp_base(double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid);
    void LRL_base(  );
    void RLR_base(  );

    void LpRupLumRm_base(double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid);

    void LpRumLumRp_base(double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid);

    void LpSpLp(   );
    void LpSpRp(   );
    void LmSmLm(   );
    void LmSmRm(   );
    void RpSpRp(   );
    void RpSpLp(   );
    void RmSmRm(   );
    void RmSmLm(   );

    // void LpRmL (   );
    // void LpRmLp(   );
    // void LmRpLm(   );
    // void RpLmRp(   );
    // void RmLpRm(   );
    // void LpRmLm(   );
    // void LmRpLp(   );
    // void RpLmRm(   );
    // void RmLpRp(   );

    void LpRupLumRm( );
    void LmRumLupRp( );
    void RpLupRumLm( );
    void RmLumRupLp( );

    void LpRumLumRp( );
    void LmRupLupRm( );
    void RpLumRumLp( );
    void RmLupRupLm( );


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

    // these part is for CCC path, using dubins formula 
    double dyy = goal_pose[1] - start_pose[1];
    double dxx = goal_pose[0] - start_pose[0];
    double theta = std::atan2(dyy,dxx);
    double D = sqrt( dxx*dxx + dyy*dyy );
    ccc_d_ = D/turning_raius_;
    ccc_alpha_ = rectify_angle_rad( start_pose[2] - theta );
    ccc_beta_  = rectify_angle_rad( goal_pose[2] - theta );

    all_possible_paths_.reset(); 

    std::cout << "\npaths reset" << std::endl;  

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

void ReedsSheppClass::tauOmega(double u, double v, double xi, double eta, double phi, double &tau, double &omega)
{
    double delta = mod2pi(u - v), A = sin(u) - sin(delta), B = cos(u) - cos(delta) - 1.;
    double t1 = atan2(eta * A - xi * B, xi * A + eta * B), t2 = 2. * (cos(delta) - cos(v) - cos(u)) + 3;
    tau = (t2 < 0) ? mod2pi(t1 + M_PI) : mod2pi(t1);
    omega = mod2pi(tau - u + v - phi);
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

    LRL_base(  );
    RLR_base(  );

    // LpRmL();
    // LpRmLp( );
    // LmRpLm();
    // RpLmRp();
    // RmLpRm();

    // LpRmLm();
    // LmRpLp( );
    // RpLmRm( );
    // RmLpRp( );

    LpRupLumRm( );
    LmRumLupRp( );
    RpLupRumLm( );
    RmLumRupLp( );

    LpRumLumRp( );
    LmRupLupRm( );
    RpLumRumLp( );
    RmLupRupLm( );

}

///////////////////////////   CSC 
// 8.1 in paper 
void ReedsSheppClass::LpSpLp_base( double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid )
{
    valid = false;

    polar( x-sin(phi), y-1+cos(phi), u, t );
    v = mod2pi(phi - t);
    L = std::abs(t) + std::abs(u) + std::abs(v) ;
    if( t > 0.0 && v > 0.0){
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
    LpSpRp_base(x,y,phi,t,u,v,L,valid);
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



void ReedsSheppClass::LRL_base( ){
    double d = ccc_d_; double beta = ccc_beta_;  double alpha = ccc_alpha_;
    double u = rectify_angle_rad( 2*M_PI - acos(  ( 6-d*d +2*cos(alpha-beta) + 2*d*(-sin(alpha) + sin(beta)) )/8.0 ) ) ;
    double t = rectify_angle_rad( -alpha - std::atan( ( cos(alpha) - cos(beta)) / (d+sin(alpha)-sin(beta))) + u/2.0 ) ;
    double v = rectify_angle_rad(beta) - alpha -t +  rectify_angle_rad(u);

    bool valid = false;
    // double L = std::abs(t) + std::abs(u) + std::abs(v) ;

    if( t>=0.0 && u>=0.0 && v>=0.0) valid = true;
    else{
        valid = false;
        return;
    } 

    std::cout << "LRL_base  " << valid  << "  " << t << "  " << u << "  " << v   << std::endl;

    double new_t, new_u, new_v;

    all_possible_paths_.LpRpLp.path_word = "LpRpLp";
    all_possible_paths_.LpRpLp.valid = valid;
    double robot_x     = start_pose_[0] ;
    double robot_y     = start_pose_[1] ;
    double robot_theta = start_pose_[2] ;
    new_t = t; new_u = u; new_v = v;
    all_possible_paths_.LpRpLp.path_length_unitless = std::abs(new_t) + std::abs(new_u) + std::abs(new_v) ;
    get_samples_L( new_t, robot_theta, robot_x, robot_y, all_possible_paths_.LpRpLp);
    get_samples_R( new_u, robot_theta, robot_x, robot_y, all_possible_paths_.LpRpLp);
    get_samples_L( new_v, robot_theta, robot_x, robot_y, all_possible_paths_.LpRpLp);

    all_possible_paths_.LpRpLm.path_word = "LpRpLm";
    all_possible_paths_.LpRpLm.valid = valid;
    robot_x     = start_pose_[0] ;
    robot_y     = start_pose_[1] ;
    robot_theta = start_pose_[2] ;
    new_t = t; new_u = u; new_v = v-M_PI*2.0;    
    all_possible_paths_.LpRpLm.path_length_unitless = std::abs(new_t) + std::abs(new_u) + std::abs(new_v) ;
    get_samples_L( new_t, robot_theta, robot_x, robot_y, all_possible_paths_.LpRpLm);
    get_samples_R( new_u, robot_theta, robot_x, robot_y, all_possible_paths_.LpRpLm);
    get_samples_L( new_v, robot_theta, robot_x, robot_y, all_possible_paths_.LpRpLm);
    
    all_possible_paths_.LpRmLp.path_word = "LpRmLp";
    all_possible_paths_.LpRmLp.valid = valid;
    robot_x     = start_pose_[0] ;
    robot_y     = start_pose_[1] ;
    robot_theta = start_pose_[2] ;
    new_t = t; new_u = u-M_PI*2.0; new_v = v;
    all_possible_paths_.LpRmLp.path_length_unitless = std::abs(new_t) + std::abs(new_u) + std::abs(new_v) ;
    get_samples_L( new_t, robot_theta, robot_x, robot_y, all_possible_paths_.LpRmLp);
    get_samples_R( new_u, robot_theta, robot_x, robot_y, all_possible_paths_.LpRmLp);
    get_samples_L( new_v, robot_theta, robot_x, robot_y, all_possible_paths_.LpRmLp);

    all_possible_paths_.LpRmLm.path_word = "LpRmLm";
    all_possible_paths_.LpRmLm.valid = valid;
    robot_x     = start_pose_[0] ;
    robot_y     = start_pose_[1] ;
    robot_theta = start_pose_[2] ;
    new_t = t; new_u = u-M_PI*2.0; new_v = v-M_PI*2.0;
    all_possible_paths_.LpRmLm.path_length_unitless = std::abs(new_t) + std::abs(new_u) + std::abs(new_v) ;
    get_samples_L( new_t, robot_theta, robot_x, robot_y, all_possible_paths_.LpRmLm);
    get_samples_R( new_u, robot_theta, robot_x, robot_y, all_possible_paths_.LpRmLm);
    get_samples_L( new_v, robot_theta, robot_x, robot_y, all_possible_paths_.LpRmLm);


    all_possible_paths_.LmRpLp.path_word = "LmRpLp";
    all_possible_paths_.LmRpLp.valid = valid;
    robot_x     = start_pose_[0] ;
    robot_y     = start_pose_[1] ;
    robot_theta = start_pose_[2] ;
    new_t = t-M_PI*2.0; new_u = u; new_v = v;
    all_possible_paths_.LmRpLp.path_length_unitless = std::abs(new_t) + std::abs(new_u) + std::abs(new_v) ;
    get_samples_L( new_t, robot_theta, robot_x, robot_y, all_possible_paths_.LmRpLp);
    get_samples_R( new_u, robot_theta, robot_x, robot_y, all_possible_paths_.LmRpLp);
    get_samples_L( new_v, robot_theta, robot_x, robot_y, all_possible_paths_.LmRpLp);

    all_possible_paths_.LmRpLm.path_word = "LmRpLm";
    all_possible_paths_.LmRpLm.valid = valid;
    robot_x     = start_pose_[0] ;
    robot_y     = start_pose_[1] ;
    robot_theta = start_pose_[2] ;
    new_t = t-M_PI*2.0; new_u = u; new_v = v-M_PI*2.0;    
    all_possible_paths_.LmRpLm.path_length_unitless = std::abs(new_t) + std::abs(new_u) + std::abs(new_v) ;
    get_samples_L( new_t, robot_theta, robot_x, robot_y, all_possible_paths_.LmRpLm);
    get_samples_R( new_u, robot_theta, robot_x, robot_y, all_possible_paths_.LmRpLm);
    get_samples_L( new_v, robot_theta, robot_x, robot_y, all_possible_paths_.LmRpLm);
    
    all_possible_paths_.LmRmLp.path_word = "LmRmLp";
    all_possible_paths_.LmRmLp.valid = valid;
    robot_x     = start_pose_[0] ;
    robot_y     = start_pose_[1] ;
    robot_theta = start_pose_[2] ;
    new_t = t-M_PI*2.0; new_u = u-M_PI*2.0; new_v = v;
    all_possible_paths_.LmRmLp.path_length_unitless = std::abs(new_t) + std::abs(new_u) + std::abs(new_v) ;
    get_samples_L( new_t, robot_theta, robot_x, robot_y, all_possible_paths_.LmRmLp);
    get_samples_R( new_u, robot_theta, robot_x, robot_y, all_possible_paths_.LmRmLp);
    get_samples_L( new_v, robot_theta, robot_x, robot_y, all_possible_paths_.LmRmLp);

    all_possible_paths_.LmRmLm.path_word = "LmRmLm";
    all_possible_paths_.LmRmLm.valid = valid;
    robot_x     = start_pose_[0] ;
    robot_y     = start_pose_[1] ;
    robot_theta = start_pose_[2] ;
    new_t = t-M_PI*2.0; new_u = u-M_PI*2.0; new_v = v-M_PI*2.0;
    all_possible_paths_.LmRmLm.path_length_unitless = std::abs(new_t) + std::abs(new_u) + std::abs(new_v) ;
    get_samples_L( new_t, robot_theta, robot_x, robot_y, all_possible_paths_.LmRmLm);
    get_samples_R( new_u, robot_theta, robot_x, robot_y, all_possible_paths_.LmRmLm);
    get_samples_L( new_v, robot_theta, robot_x, robot_y, all_possible_paths_.LmRmLm);

}



void ReedsSheppClass::RLR_base( ){
    double d = ccc_d_; double beta = ccc_beta_;  double alpha = ccc_alpha_;
    double u = rectify_angle_rad( 2*M_PI - acos(  ( 6-d*d +2*cos(alpha-beta) + 2*d*(sin(alpha) - sin(beta)) )/8.0 ) ) ;
    double t = rectify_angle_rad( alpha - std::atan( ( cos(alpha) - cos(beta)) / (d-sin(alpha)+sin(beta))) + rectify_angle_rad(u/2.0) ) ;
    double v = rectify_angle_rad( -beta + alpha -t +  rectify_angle_rad(u));

    bool valid = false;
    if( t>=0.0 && u>=0.0 && v>=0.0) valid = true;
    else{
        valid = false;
        return;
    } 

    std::cout << "RLR_base  " << valid  << "  " << t << "  " << u << "  " << v   << std::endl;

    double new_t, new_u, new_v;

    all_possible_paths_.RpLpRp.path_word = "RpLpRp";
    all_possible_paths_.RpLpRp.valid = valid;
    double robot_x     = start_pose_[0] ;
    double robot_y     = start_pose_[1] ;
    double robot_theta = start_pose_[2] ;
    new_t = t; new_u = u; new_v = v;
    all_possible_paths_.RpLpRp.path_length_unitless = std::abs(new_t) + std::abs(new_u) + std::abs(new_v) ;
    get_samples_R( new_t, robot_theta, robot_x, robot_y, all_possible_paths_.RpLpRp);
    get_samples_L( new_u, robot_theta, robot_x, robot_y, all_possible_paths_.RpLpRp);
    get_samples_R( new_v, robot_theta, robot_x, robot_y, all_possible_paths_.RpLpRp);

    all_possible_paths_.RpLpRm.path_word = "RpLpRm";
    all_possible_paths_.RpLpRm.valid = valid;
    robot_x     = start_pose_[0] ;
    robot_y     = start_pose_[1] ;
    robot_theta = start_pose_[2] ;
    new_t = t; new_u = u; new_v = v-M_PI*2.0;
    all_possible_paths_.RpLpRm.path_length_unitless = std::abs(new_t) + std::abs(new_u) + std::abs(new_v) ;
    get_samples_R( new_t, robot_theta, robot_x, robot_y, all_possible_paths_.RpLpRm);
    get_samples_L( new_u, robot_theta, robot_x, robot_y, all_possible_paths_.RpLpRm);
    get_samples_R( new_v, robot_theta, robot_x, robot_y, all_possible_paths_.RpLpRm);

    all_possible_paths_.RpLmRp.path_word = "RpLmRp";
    all_possible_paths_.RpLmRp.valid = valid;
    robot_x     = start_pose_[0] ;
    robot_y     = start_pose_[1] ;
    robot_theta = start_pose_[2] ;
    new_t = t; new_u = u-M_PI*2.0; new_v = v;
    all_possible_paths_.RpLmRp.path_length_unitless = std::abs(new_t) + std::abs(new_u) + std::abs(new_v) ;
    get_samples_R( new_t, robot_theta, robot_x, robot_y, all_possible_paths_.RpLmRp);
    get_samples_L( new_u, robot_theta, robot_x, robot_y, all_possible_paths_.RpLmRp);
    get_samples_R( new_v, robot_theta, robot_x, robot_y, all_possible_paths_.RpLmRp);

    all_possible_paths_.RpLmRm.path_word = "RpLmRm";
    all_possible_paths_.RpLmRm.valid = valid;
    robot_x     = start_pose_[0] ;
    robot_y     = start_pose_[1] ;
    robot_theta = start_pose_[2] ;
    new_t = t; new_u = u-M_PI*2.0; new_v = v-M_PI*2.0;
    all_possible_paths_.RpLmRm.path_length_unitless = std::abs(new_t) + std::abs(new_u) + std::abs(new_v) ;
    get_samples_R( new_t, robot_theta, robot_x, robot_y, all_possible_paths_.RpLmRm);
    get_samples_L( new_u, robot_theta, robot_x, robot_y, all_possible_paths_.RpLmRm);
    get_samples_R( new_v, robot_theta, robot_x, robot_y, all_possible_paths_.RpLmRm);


    all_possible_paths_.RmLpRp.path_word = "RmLpRp";
    all_possible_paths_.RmLpRp.valid = valid;
    robot_x     = start_pose_[0] ;
    robot_y     = start_pose_[1] ;
    robot_theta = start_pose_[2] ;
    new_t = t-M_PI*2.0; new_u = u; new_v = v;
    all_possible_paths_.RmLpRp.path_length_unitless = std::abs(new_t) + std::abs(new_u) + std::abs(new_v) ;
    get_samples_R( new_t, robot_theta, robot_x, robot_y, all_possible_paths_.RmLpRp);
    get_samples_L( new_u, robot_theta, robot_x, robot_y, all_possible_paths_.RmLpRp);
    get_samples_R( new_v, robot_theta, robot_x, robot_y, all_possible_paths_.RmLpRp);

    all_possible_paths_.RmLpRm.path_word = "RmLpRm";
    all_possible_paths_.RmLpRm.valid = valid;
    robot_x     = start_pose_[0] ;
    robot_y     = start_pose_[1] ;
    robot_theta = start_pose_[2] ;
    new_t = t-M_PI*2.0; new_u = u; new_v = v-M_PI*2.0;
    all_possible_paths_.RmLpRm.path_length_unitless = std::abs(new_t) + std::abs(new_u) + std::abs(new_v) ;
    get_samples_R( new_t, robot_theta, robot_x, robot_y, all_possible_paths_.RmLpRm);
    get_samples_L( new_u, robot_theta, robot_x, robot_y, all_possible_paths_.RmLpRm);
    get_samples_R( new_v, robot_theta, robot_x, robot_y, all_possible_paths_.RmLpRm);

    all_possible_paths_.RmLmRp.path_word = "RmLmRp";
    all_possible_paths_.RmLmRp.valid = valid;
    robot_x     = start_pose_[0] ;
    robot_y     = start_pose_[1] ;
    robot_theta = start_pose_[2] ;
    new_t = t-M_PI*2.0; new_u = u-M_PI*2.0; new_v = v;
    all_possible_paths_.RmLmRp.path_length_unitless = std::abs(new_t) + std::abs(new_u) + std::abs(new_v) ;
    get_samples_R( new_t, robot_theta, robot_x, robot_y, all_possible_paths_.RmLmRp);
    get_samples_L( new_u, robot_theta, robot_x, robot_y, all_possible_paths_.RmLmRp);
    get_samples_R( new_v, robot_theta, robot_x, robot_y, all_possible_paths_.RmLmRp);

    all_possible_paths_.RmLmRm.path_word = "RmLmRm";
    all_possible_paths_.RmLmRm.valid = valid;
    robot_x     = start_pose_[0] ;
    robot_y     = start_pose_[1] ;
    robot_theta = start_pose_[2] ;
    new_t = t-M_PI*2.0; new_u = u-M_PI*2.0; new_v = v-M_PI*2.0;
    all_possible_paths_.RmLmRm.path_length_unitless = std::abs(new_t) + std::abs(new_u) + std::abs(new_v) ;
    get_samples_R( new_t, robot_theta, robot_x, robot_y, all_possible_paths_.RmLmRm);
    get_samples_L( new_u, robot_theta, robot_x, robot_y, all_possible_paths_.RmLmRm);
    get_samples_R( new_v, robot_theta, robot_x, robot_y, all_possible_paths_.RmLmRm);

}



// 8.7
void ReedsSheppClass::LpRupLumRm_base(double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid)
{
    double xi = x + sin(phi);
    double eta = y - 1.0 - cos(phi);
    double rho = 0.25 * (2.0 + sqrt(xi * xi + eta * eta));

    if ( rho > 1.0 || rho < 0.0 ){
        valid = false;
        return;
    }
    valid = false;
    u = acos(rho);
    tauOmega(u, -u, xi, eta, phi, t, v);
    if (t >= 0.0 && v <= 0.0 ){
        valid = true;
    }
}


void ReedsSheppClass::LpRupLumRm( ){
    double x   = goal_pose_processed_[0];
    double y   = goal_pose_processed_[1];
    double phi = goal_pose_processed_[2];
    double t, u, v, L; bool valid;
    LpRupLumRm_base( x, y, phi, t, u, v, L, valid);
    all_possible_paths_.LpRupLumRm.path_word = "LpRupLumRm";
    all_possible_paths_.LpRupLumRm.valid = valid;
    if( valid ){
        L = std::abs(t) + std::abs(u)*2.0 + std::abs(v) ; 
        all_possible_paths_.LpRupLumRm.path_length_unitless = L; 
        double robot_x     = start_pose_[0] ;
        double robot_y     = start_pose_[1] ;
        double robot_theta = start_pose_[2] ;
        get_samples_L( t, robot_theta, robot_x, robot_y, all_possible_paths_.LpRupLumRm);
        get_samples_R( u, robot_theta, robot_x, robot_y, all_possible_paths_.LpRupLumRm);
        get_samples_L( -u, robot_theta, robot_x, robot_y, all_possible_paths_.LpRupLumRm);
        get_samples_R( v, robot_theta, robot_x, robot_y, all_possible_paths_.LpRupLumRm);
    }
    // if(valid) std::cout << "LpRupLumRm  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
}


void ReedsSheppClass::LmRumLupRp( ){
    double x   = goal_pose_processed_[0];
    double y   = goal_pose_processed_[1];
    double phi = goal_pose_processed_[2];
    double t, u, v, L; bool valid;
    LpRupLumRm_base( -x, y, -phi, t, u, v, L, valid);
    all_possible_paths_.LmRumLupRp.path_word = "LmRumLupRp";
    all_possible_paths_.LmRumLupRp.valid = valid;
    if( valid ){
        L = std::abs(t) + std::abs(u)*2.0 + std::abs(v) ; 
        all_possible_paths_.LmRumLupRp.path_length_unitless = L; 
        double robot_x     = start_pose_[0] ;
        double robot_y     = start_pose_[1] ;
        double robot_theta = start_pose_[2] ;
        get_samples_L( -t, robot_theta, robot_x, robot_y, all_possible_paths_.LmRumLupRp);
        get_samples_R( -u, robot_theta, robot_x, robot_y, all_possible_paths_.LmRumLupRp);
        get_samples_L( u, robot_theta, robot_x, robot_y, all_possible_paths_.LmRumLupRp);
        get_samples_R( -v, robot_theta, robot_x, robot_y, all_possible_paths_.LmRumLupRp);
    }
    // if(valid) std::cout << "LmRumLupRp  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
}


void ReedsSheppClass::RpLupRumLm( ){
    double x   = goal_pose_processed_[0];
    double y   = goal_pose_processed_[1];
    double phi = goal_pose_processed_[2];
    double t, u, v, L; bool valid;
    LpRupLumRm_base( x, -y, -phi, t, u, v, L, valid);
    all_possible_paths_.RpLupRumLm.path_word = "RpLupRumLm";
    all_possible_paths_.RpLupRumLm.valid = valid;
    if( valid ){
        L = std::abs(t) + std::abs(u)*2.0 + std::abs(v) ; 
        all_possible_paths_.RpLupRumLm.path_length_unitless = L; 
        double robot_x     = start_pose_[0] ;
        double robot_y     = start_pose_[1] ;
        double robot_theta = start_pose_[2] ;
        get_samples_R( t, robot_theta, robot_x, robot_y, all_possible_paths_.RpLupRumLm);
        get_samples_L( u, robot_theta, robot_x, robot_y, all_possible_paths_.RpLupRumLm);
        get_samples_R( -u, robot_theta, robot_x, robot_y, all_possible_paths_.RpLupRumLm);
        get_samples_L( v, robot_theta, robot_x, robot_y, all_possible_paths_.RpLupRumLm);
    }
    // if(valid) std::cout << "RpLupRumLm  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
}


void ReedsSheppClass::RmLumRupLp( ){
    double x   = goal_pose_processed_[0];
    double y   = goal_pose_processed_[1];
    double phi = goal_pose_processed_[2];
    double t, u, v, L; bool valid;
    LpRupLumRm_base( -x, -y, phi, t, u, v, L, valid);
    all_possible_paths_.RmLumRupLp.path_word = "RmLumRupLp";
    all_possible_paths_.RmLumRupLp.valid = valid;
    if( valid ){
        L = std::abs(t) + std::abs(u)*2.0 + std::abs(v) ; 
        all_possible_paths_.RmLumRupLp.path_length_unitless = L; 
        double robot_x     = start_pose_[0] ;
        double robot_y     = start_pose_[1] ;
        double robot_theta = start_pose_[2] ;
        get_samples_R( -t, robot_theta, robot_x, robot_y, all_possible_paths_.RmLumRupLp);
        get_samples_L( -u, robot_theta, robot_x, robot_y, all_possible_paths_.RmLumRupLp);
        get_samples_R( u, robot_theta, robot_x, robot_y, all_possible_paths_.RmLumRupLp);
        get_samples_L( -v, robot_theta, robot_x, robot_y, all_possible_paths_.RmLumRupLp);
    }
    // if(valid) std::cout << "RmLumRupLp  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
}


// 8.8
void ReedsSheppClass::LpRumLumRp_base(double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid)
{
    double xi = x + sin(phi);
    double eta = y - 1.0 - cos(phi);
    double rho = (20.0 - xi * xi - eta * eta) / 16.;


    // if ( rho > 1.0 || rho < 0.0 ){
    //     valid = false;
    //     return;
    // }
    // valid = false;
    // u = -acos(rho);
    // if ( std::abs(u) > M_PI*0.5 ){
    //     valid = false;
    //     return;
    // }
    // tauOmega(u, -u, xi, eta, phi, t, v);
    // if (t >= 0.0 && v >= 0.0 ){
    //     valid = true;
    // }
    
    valid = false;
    if (rho >= 0 && rho <= 1)
    {
        u = -acos(rho);
        if (u >= -.5 * M_PI)
        {
            tauOmega(u, u, xi, eta, phi, t, v);
            // assert(fabs(4 * sin(t) - 2 * sin(t - u) - sin(phi) - x) < RS_EPS);
            // assert(fabs(-4 * cos(t) + 2 * cos(t - u) + cos(phi) + 1 - y) < RS_EPS);
            // assert(fabs(mod2pi(t - v - phi)) < RS_EPS);
            valid = (t >= 0.0 && v >= 0.0);
        }
    }

}


void ReedsSheppClass::LpRumLumRp( )
{
    double x   = goal_pose_processed_[0];
    double y   = goal_pose_processed_[1];
    double phi = goal_pose_processed_[2];
    double t, u, v, L; bool valid;
    LpRumLumRp_base( x, y, phi, t, u, v, L, valid);
    all_possible_paths_.LpRumLumRp.path_word = "LpRumLumRp";
    all_possible_paths_.LpRumLumRp.valid = valid;
    if( valid ){
        L = std::abs(t) + std::abs(u)*2.0 + std::abs(v) ; 
        all_possible_paths_.LpRumLumRp.path_length_unitless = L; 
        double robot_x     = start_pose_[0] ;
        double robot_y     = start_pose_[1] ;
        double robot_theta = start_pose_[2] ;
        get_samples_L( t, robot_theta, robot_x, robot_y, all_possible_paths_.LpRumLumRp);
        get_samples_R( u, robot_theta, robot_x, robot_y, all_possible_paths_.LpRumLumRp);
        get_samples_L( u, robot_theta, robot_x, robot_y, all_possible_paths_.LpRumLumRp);
        get_samples_R( v, robot_theta, robot_x, robot_y, all_possible_paths_.LpRumLumRp);
    }
    // if(valid) std::cout << "LpRumLumRp  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
}

void ReedsSheppClass::LmRupLupRm( )
{
    double x   = goal_pose_processed_[0];
    double y   = goal_pose_processed_[1];
    double phi = goal_pose_processed_[2];
    double t, u, v, L; bool valid;
    LpRumLumRp_base( -x, y, -phi, t, u, v, L, valid);
    all_possible_paths_.LmRupLupRm.path_word = "LmRupLupRm";
    all_possible_paths_.LmRupLupRm.valid = valid;
    if( valid ){
        L = std::abs(t) + std::abs(u)*2.0 + std::abs(v) ; 
        all_possible_paths_.LmRupLupRm.path_length_unitless = L; 
        double robot_x     = start_pose_[0] ;
        double robot_y     = start_pose_[1] ;
        double robot_theta = start_pose_[2] ;
        get_samples_L( -t, robot_theta, robot_x, robot_y, all_possible_paths_.LmRupLupRm);
        get_samples_R( -u, robot_theta, robot_x, robot_y, all_possible_paths_.LmRupLupRm);
        get_samples_L( -u, robot_theta, robot_x, robot_y, all_possible_paths_.LmRupLupRm);
        get_samples_R( -v, robot_theta, robot_x, robot_y, all_possible_paths_.LmRupLupRm);
    }
    // if(valid) std::cout << "LmRupLupRm  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
}

void ReedsSheppClass::RpLumRumLp( )
{
    double x   = goal_pose_processed_[0];
    double y   = goal_pose_processed_[1];
    double phi = goal_pose_processed_[2];
    double t, u, v, L; bool valid;
    LpRumLumRp_base( x, -y, -phi, t, u, v, L, valid);
    all_possible_paths_.RpLumRumLp.path_word = "RpLumRumLp";
    all_possible_paths_.RpLumRumLp.valid = valid;
    if( valid ){
        L = std::abs(t) + std::abs(u)*2.0 + std::abs(v) ; 
        all_possible_paths_.RpLumRumLp.path_length_unitless = L; 
        double robot_x     = start_pose_[0] ;
        double robot_y     = start_pose_[1] ;
        double robot_theta = start_pose_[2] ;
        get_samples_R( t, robot_theta, robot_x, robot_y, all_possible_paths_.RpLumRumLp);
        get_samples_L( u, robot_theta, robot_x, robot_y, all_possible_paths_.RpLumRumLp);
        get_samples_R( u, robot_theta, robot_x, robot_y, all_possible_paths_.RpLumRumLp);
        get_samples_L( v, robot_theta, robot_x, robot_y, all_possible_paths_.RpLumRumLp);
    }
    // if(valid) std::cout << "RpLumRumLp  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
}

void ReedsSheppClass::RmLupRupLm( )
{
    double x   = goal_pose_processed_[0];
    double y   = goal_pose_processed_[1];
    double phi = goal_pose_processed_[2];
    double t, u, v, L; bool valid;
    LpRumLumRp_base( -x, -y, phi, t, u, v, L, valid);
    all_possible_paths_.RmLupRupLm.path_word = "RmLupRupLm";
    all_possible_paths_.RmLupRupLm.valid = valid;
    if( valid ){
        L = std::abs(t) + std::abs(u)*2.0 + std::abs(v) ; 
        all_possible_paths_.RmLupRupLm.path_length_unitless = L; 
        double robot_x     = start_pose_[0] ;
        double robot_y     = start_pose_[1] ;
        double robot_theta = start_pose_[2] ;
        get_samples_R( -t, robot_theta, robot_x, robot_y, all_possible_paths_.RmLupRupLm);
        get_samples_L( -u, robot_theta, robot_x, robot_y, all_possible_paths_.RmLupRupLm);
        get_samples_R( -u, robot_theta, robot_x, robot_y, all_possible_paths_.RmLupRupLm);
        get_samples_L( -v, robot_theta, robot_x, robot_y, all_possible_paths_.RmLupRupLm);
    }
    // if(valid) std::cout << "RmLupRupLm  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
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