
#ifndef ASTAR_H
#define ASTAR_H

#include <iostream>
#include <queue>
#include <unordered_map>
#include <map>
#include <opencv2/core.hpp>
#include <algorithm>
#include <set>

#include "utils/array_hasher.cpp"



class astar
{
private:
    std::vector< std::vector<float>> motion_model;

    std::array<int, 3> start_grid;
    std::array<float, 3> start_pose;

    std::array<int, 3> goal_grid;
    std::array<float, 3> goal_pose;

    std::array<float, 3> close_goal_pose;

    float start_angle, goal_angle, close_goal_angle; 

    std::unordered_map<std::array<int, 3>, std::array<int,   3>, ArrayHasher3> grid_parent;
    std::unordered_map<std::array<int, 3>, std::array<float, 3>, ArrayHasher3> grid_fine_pose;
    std::unordered_map<std::array<int, 3>, std::array<int,   4>, ArrayHasher3> grid_status;
    /*
    0 : gcost.  
    1 : fcost.
    2 : state :  0:new   1:open   2:closed
    3 : steer_type:  0-5 
    */


    int fine_map_width, fine_map_height, grid_map_width, grid_map_height;

    cv::Mat map_grid;     // low resolution map
    cv::Mat map_fine;     // fine resolution map
    cv::Mat map_for_view; // the copy used for visulization, constantly being modiified
    cv::Mat map_collision; // the copy used for check collision

    const float fine_ratio = 10;
    const float angle_resolution = M_PI/18.0; // radius
    const int obstacle_scan_range = 15;
    const int map_inflation_size = 5;

    const std::string window_name = "map";

    const float step_length = 20;
    const std::array<float, 2> turning_raius = {25, 40 };
    std::array<float, 2> turning_angle ;

    bool FLAG_reach_goal = false;

    void build_motion_model();
    void map_inflation(cv::Mat* mapIn, cv::Mat* mapOut, int infla_size);
    void print_init_info();

    void draw_start_goal_on_map();
    void draw_arc(std::array<float, 3> node_pose, std::array<float, 3> next_pose, int radius, float arc_angle, int motion_type , int thickness, cv::Scalar color);
    
    int check_collision( float x, float y, int scan_range );
    int improve_hcost(std::array<float, 3> node_pose, int hcost, int scan_range, bool same_steer);
    void check_if_reach_goal(std::array<float, 3> node_pose,std::array<float, 3> in_goal_pose, std::array<int, 3> in_curr_grid);

public:
    astar(int startnode[], int goalnode[], float s_angle, float g_angle, cv::Mat map);

    ~astar();

    bool FLAG_update_map_for_view = true;

    std::deque< std::array<int,3>> path;


    void explore_one_node(std::array<float, 3> curr_pose, std::array<int, 3> curr_grid);
    void explore_one_ite(std::vector< std::array<int, 3>> active_nodes);
    std::vector< std::array<int,3>> find_min_cost_nodes();
    void search();
    void get_path();
    void print_path();


};


#endif