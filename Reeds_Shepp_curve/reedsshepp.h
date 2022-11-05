
#ifndef REEDSSHEPP_H
#define REEDSSHEPP_H

#include <iostream>
#include <queue>
#include <unordered_map>
#include <map>
#include <opencv2/core.hpp>
#include <algorithm>
#include <set>

#include "utils/array_hasher.cpp"



class ReedsSheppClass
{
private:
    // std::vector< std::vector<float>> motion_model;

    std::array<float, 3> start_pose;
    std::array<float, 3> goal_pose;

    float turning_raius = 40;

    int fine_map_width, fine_map_height, grid_map_width, grid_map_height;

    cv::Mat map_fine;     // fine resolution map

    cv::Mat map_for_view; // the copy used for visulization, constantly being modified

    cv::Mat map_collision; // the copy used for check collision

    const float fine_ratio = 10;
    const int obstacle_scan_range = 15;
    const int map_inflation_size = 5;

    struct posePerSample
    {
        posePerSample(float xp, float yp, float angle):x(xp),y(yp),theta(angle){};
        float x;
        float y;
        float theta; // radian
    };
    

    struct pathResult
    {
        float path_length;
        std::vector< posePerSample > path_steps;
    };
    

    float angular_step_size = 0.05;
    float linear_step_size = 2;

    float v_l ;

    std::unordered_map<std::string, pathResult> all_path_result;

    std::unordered_map<std::string, cv::Scalar> all_path_colors;

    const std::string window_name = "map";

    // const float step_length = 20;
    // std::array<float, 2> turning_angle ;

    // bool FLAG_reach_goal = false;

    // void build_motion_model();
    void map_inflation(cv::Mat* mapIn, cv::Mat* mapOut, int infla_size);
    // void print_init_info();

    void polar(double x, double y, double &radius_ro, double &theta);

    double mod2pi(double angle_radian);
    const double pi2_ = M_PI*2.0;  

    const double ZERO = 10 * std::numeric_limits<double>::epsilon();
    const double RS_EPS = 1e-6; 

    float rectify_angle_rad(float ang);
    float compute_h_cost_Euclidean(const std::array<float, 3> n, const std::array<float, 3> g);

    void draw_start_goal_on_map( int arrow_length  );
    void draw_vector_on_map(std::array<float, 3> vect);
    // void draw_arc(std::array<float, 3> node_pose, std::array<float, 3> next_pose, int radius, float arc_angle, int motion_type , int thickness, cv::Scalar color);
    
    int check_collision( float x, float y, int scan_range );
    // int improve_hcost(std::array<float, 3> node_pose, int hcost, int scan_range, bool same_steer);
    // void check_if_reach_goal(std::array<float, 3> node_pose,std::array<float, 3> in_goal_pose, std::array<int, 3> in_curr_grid);

    // void compute_circle_center(std::array<float,3> robot_pose, std::string LorR, int radius, std::array<float,2>& center );

    void sample_points_on_path();

    void LpSpLp(double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid);
    void LpSpRp(double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid);
    void LmSmLm(double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid);
    void LmSmRm(double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid);
    void RpSpRp(double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid);
    void RpSpLp(double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid);
    void RmSmRm(double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid);
    void RmSmLm(double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid);

    void LpRmL(double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid);
    void LmRpLm(double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid);
    void RpLmRp(double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid);
    void RmLpRm(double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid);
    void LpRmLm(double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid);
    void LmRpLp(double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid);
    void RpLmRm(double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid);
    void RmLpRp(double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid);

    // void get_samples_L( const float target_angle_change, float robot_yaw, float& robotx, float& roboty, const std::string path_type  );
    void get_samples_L( const float target_angle_change, float& robot_yaw, float& robotx, float& roboty, const std::string path_type  );
    void get_samples_R( const float target_angle_change, float& robot_yaw, float& robotx, float& roboty, const std::string path_type  );
    void get_samples_S( const float target_angle_change, float robot_yaw, float& robotx, float& roboty, const std::string path_type  );

public:
    ReedsSheppClass();

    ~ReedsSheppClass();

    void setup(int startnode[], int goalnode[], float s_angle, float g_angle, cv::Mat map);

    bool FLAG_update_map_for_view = true;

    std::deque< std::array<int,3>> path;


    // void search( std::array<float,3> start_pose, std::array<float,3> goal_pose);

    void search( bool last  );

    void get_path( std::array<float,3> start_pose, std::array<float,3> goal_pose , std::string path_type, std::vector<float>& path_angle, bool& valid);

    // void compute_LSL_path(float alpha, float beta, float d, std::vector<float>& path_angles_seq, float& total_length);
    // void compute_RSR_path(float alpha, float beta, float d, std::vector<float>& path_angles_seq, float& total_length);
    // void compute_LSR_path(float alpha, float beta, float d, std::vector<float>& path_angles_seq, float& total_length);
    // void compute_RSL_path(float alpha, float beta, float d, std::vector<float>& path_angles_seq, float& total_length);
    // void compute_LRL_path(float alpha, float beta, float d, std::vector<float>& path_angles_seq, float& total_length);
    // void compute_RLR_path(float alpha, float beta, float d, std::vector<float>& path_angles_seq, float& total_length);

    void print_path();


};


#endif