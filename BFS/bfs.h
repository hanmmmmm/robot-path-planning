
#ifndef BFS_H
#define BFS_H

#include <iostream>
#include <queue>
#include <unordered_map>
#include <map>
#include <opencv2/core.hpp>
#include <algorithm>
#include <set>
#include "utils/array_hasher.cpp"



class bfs
{
private:
    std::queue<std::array<int,2>> q;

    int motion_model[8][2];

    std::array<int, 2> start_xy;
    std::array<int, 2> goal_xy;
    
    std::array<int, 2> curr_xy;

    int map_width;
    int map_height;

    cv::Mat mapCv;
    cv::Mat map_for_view;

    void build_motion_model();
    void build_parent_grid();
    void print_init_info();
    void draw_start_goal_on_map();

public:
    bfs(int startnode[],
        int goalnode[],
        cv::Mat map);

    ~bfs();

    bool FLAG_update_map_for_view = true;

    void explore_one_node();
    void search();
    std::deque< std::array<int,2>> get_path();
    void get_path_no_return();
    void print_path();


    std::deque< std::array<int, 2>> path;

    std::unordered_map<std::array<int, 2>, std::array<int, 2>, ArrayHasher> parent_nodes;
    std::unordered_map<std::array<int, 2>, bool, ArrayHasher > visited;
};


#endif