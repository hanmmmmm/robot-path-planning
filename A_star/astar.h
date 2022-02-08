
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
    std::queue<std::array<int,2>> q;

    int motion_model[8][3];

    std::array<int, 2> start_xy;
    std::array<int, 2> goal_xy;
    
    std::array<int, 2> curr_xy;

    int map_width;
    int map_height;

    cv::Mat mapCv;
    cv::Mat map_for_view;

    std::string window_name = "map";

    void build_motion_model();
    void build_parent_grid();
    void print_init_info();
    void draw_start_goal_on_map();

    int compute_h_cost_Euclidean(std::array<int,2> n );
    int compute_h_cost_Manhattan(std::array<int,2> n );
    int compute_h_cost_Chebyshev(std::array<int,2> n );

public:
    astar(int startnode[],
        int goalnode[],
        cv::Mat map);

    ~astar();

    std::unordered_map<std::array<int, 2>, std::array<int, 2>, ArrayHasher> node_parent;
    std::unordered_map<std::array<int, 2>, int, ArrayHasher > node_state; // 0: new ; 1: open ; 2: closed
    std::unordered_map<std::array<int, 2>, int, ArrayHasher > node_gcost;
    std::unordered_map<std::array<int, 2>, int, ArrayHasher > node_fcost;


    bool FLAG_update_map_for_view = true;

    std::deque< std::array<int, 2>> path;


    void explore_one_node(std::array<int, 2> curr_xy);
    void explore_one_ite(std::vector< std::array<int, 2>> active_nodes);
    std::vector< std::array<int, 2>> find_min_cost_nodes();
    void search();
    void get_path_no_return();
    void print_path();


};


#endif