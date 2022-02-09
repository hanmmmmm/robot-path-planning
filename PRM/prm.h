
#ifndef PRM_H
#define PRM_H

#include <iostream>
#include <queue>
#include <unordered_map>
#include <map>
#include <opencv2/core.hpp>
#include <algorithm>
#include <set>

#include "utils/array_hasher.cpp"



class Prm
{
private:
    std::array<int, 2> start_pose;
    std::array<int, 2> goal_pose;
    std::array<int, 2> close_goal_pose;

    struct nodeInfo
    {
        std::vector< std::array<int,2> > connected_nodes;
        std::vector<int> costs;
        std::array<int,2> parent;
        bool visited = false;
    };
    
    std::unordered_map<std::array<int, 2>, nodeInfo, ArrayHasher> all_nodes;

    int fine_map_width, fine_map_height;

    cv::Mat map_fine;     // fine resolution map
    cv::Mat map_for_view; // the copy used for visulization, constantly being modiified
    cv::Mat map_collision; // the copy used for check collision
    
    const float k_fine_ratio = 10;
    const int k_map_inflation_size = 5;

    const std::string k_window_name = "map";

    const float k_max_step_length = 200;

    bool FLAG_reach_goal = false;

    std::vector< std::array<int,2> > path_buff;
    std::vector< std::vector<std::array<int,2> > > all_path;
    
    void map_inflation(cv::Mat* mapIn, cv::Mat* mapOut, int infla_size);
    void print_init_info();

    void draw_start_goal_on_map();
    int check_collision_by_area( float x, float y, int scan_range );
    bool check_collision_by_line(std::array<int,2> pose1, std::array<int,2> pose2 , int num_step );
    bool check_collision_by_point(int x, int y);
    void check_if_reach_goal(std::array<float, 3> node_pose,std::array<float, 3> in_goal_pose, std::array<int, 3> in_curr_grid);

    void sample_a_point( int x, int y);

    


public:
    Prm(int startnode[], int goalnode[],  cv::Mat map);

    ~Prm();

    bool FLAG_update_map_for_view = true;

    std::deque< std::array<int,2>> path;


    void explore_one_node(std::array<float, 3> curr_pose, std::array<int, 3> curr_grid);
    void explore_one_ite(std::vector< std::array<int, 3>> active_nodes);
    
    void search();
    void get_path();
    void get_path_dfs_recursive();
    void dfs_recursive(std::array<int,2> curr, std::array<int,2> goal);
    void print_path();

    void test_collision_func();


};


#endif