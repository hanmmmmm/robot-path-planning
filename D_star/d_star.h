
#ifndef D_STAR_H
#define D_STAR_H

#include <iostream>
#include <queue>
#include <unordered_map>
#include <map>
#include <opencv2/core.hpp>
#include <algorithm>
#include <set>

#include "utils/array_hasher.cpp"



class D_star
{
private:
    // std::queue<std::array<int,2>> q;

    int motion_model[8][3];

    std::array<int, 2> start_xy;
    std::array<int, 2> goal_xy;
    
    std::array<int, 2> curr_xy;

    int map_width;
    int map_height;

    cv::Mat grid_map;
    cv::Mat map_for_view;
    cv::Mat map_collision;

    std::string window_name = "map";

    struct nodeInfo
    {
        std::array<int, 2> parent_grid;
        int g_cost;
        int k;
    };
    
    std::unordered_map<std::array<int, 2>, nodeInfo, ArrayHasher> all_nodes;
    std::set< std::array<int,2> > open_nodes;
    std::set< std::array<int,2> > close_nodes;
    int d_star_Kmin;


    void build_motion_model();
    void build_parent_grid();
    void print_init_info();
    void draw_start_goal_on_map();
    void draw_all_nodes_on_map();
    void draw_path_on_map();

    bool check_collision_by_point(int x, int y);
    bool check_valid_by_point(int x, int y);

    void d_star_INSERT(std::array<int,2> grid, int hnew);
    void d_star_DELETE(std::array<int,2> grid);
    void d_star_PROCESS_STATE();
    void d_star_MODIFY_COST(std::array<int,2> grid);
    int d_star_GET_KMIN();
    int d_star_MIN_STATE_and_GET_KMIN(  std::vector< std::array<int,2> >& Xs  );

public:
    D_star(int startnode[],
        int goalnode[],
        cv::Mat map);

    ~D_star();

    

    bool FLAG_update_map_for_view = true;

    std::deque< std::array<int, 2>> path;

    void search();
    
    void get_path();

    void update_obs_map( std::vector<int> new_obs );

    void move_robot();


};


#endif