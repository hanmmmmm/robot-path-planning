
#include <iostream>
#include "astar.h"
#include <string>
#include <stdexcept>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <limits>
#include <math.h>

#include "utils/img_io.h"

astar::astar(int startnode[],
        int goalnode[],
        cv::Mat map)
{
    mapCv = map;
    map_for_view = map.clone();

    map_width = mapCv.size().width;
    map_height = mapCv.size().height;

    start_xy[0] = startnode[0];
    start_xy[1] = startnode[1];
    goal_xy[0] = goalnode[0];
    goal_xy[1] = goalnode[1];

    draw_start_goal_on_map();

    curr_xy = start_xy;


    q.push(start_xy);
    node_parent[start_xy] = start_xy;
    node_gcost[start_xy] = 0;
    node_state[start_xy] = 1;

    build_motion_model();

    print_init_info();
}

astar::~astar()
{
}



void astar::build_motion_model(){
    int xs[8] = {-1,0,1,1,1,0,-1,-1};
    int ys[8] = {1,1,1,0,-1,-1,-1,0};
    int cost[8] = { 14,10,14,10,14,10,14,10 };
    for(int i = 0; i<8; i++){
        motion_model[i][0] = xs[i];
        motion_model[i][1] = ys[i];
        motion_model[i][2] = cost[i];
    }
}




void astar::explore_one_node(std::array<int, 2> curr_xy){
    // move it into closed 
    node_state[curr_xy] = 2;

    for(auto mm : motion_model){
        int n_x = curr_xy[0] + mm[0];
        int n_y = curr_xy[1] + mm[1];
        int edge_cost = mm[2];

        if(0<= n_x && n_x < map_width && 0<= n_y && n_y < map_height){
            if(mapCv.at<cv::Vec3b>(n_y, n_x)[0] != 0){ // if not obstacle
                std::array<int,2> n_node = {n_x, n_y};
                if(node_parent.count(n_node) == 0){
                    node_parent[n_node] = curr_xy;
                    node_state[n_node] = 1;
                    node_gcost[n_node] = node_gcost[curr_xy] + edge_cost ;
                    // int hcost = compute_h_cost_Euclidean(n_node);
                    int hcost = compute_h_cost_Manhattan(n_node);
                    // int hcost = compute_h_cost_Chebyshev(n_node);
                    node_fcost[n_node] = node_gcost[n_node] + hcost;

                    if(FLAG_update_map_for_view){
                        if(n_node != goal_xy){
                            cv::Vec3b color = {255,100,100} ;
                            map_for_view.at<cv::Vec3b>(n_y, n_x) = color;
                        }
                    }
                } 
            }
        }
    }
}



std::vector< std::array<int, 2>> astar::find_min_cost_nodes(){
    std::vector< std::array<int, 2>> out;
    int min_cost = std::numeric_limits<int>::max();
    // if a node exist in node_parent
    for(auto n : node_parent){
        std::array<int, 2> node = n.first;
        // if its state == 1  // open
        if(node_state[node] == 1){
            int cost = node_fcost[node];
            // get its cost, compare to min_cost
            if(cost < min_cost){
                min_cost = cost;
                out.clear();
                out.push_back(node);
            }
        }
    }
    return out;
}


int astar::compute_h_cost_Euclidean( std::array<int,2> n ){
    float h = 0;

    int dx = n[0] - goal_xy[0] ;
    int dy = n[1] - goal_xy[1] ;

    h = sqrt( dx*dx + dy*dy ) *10;

    return int(h) ;
}

int astar::compute_h_cost_Manhattan( std::array<int,2> n ){
    int h = 0;

    int dx = abs( n[0] - goal_xy[0] );
    int dy = abs( n[1] - goal_xy[1] );

    h = ( dx + dy ) *10;

    return int(h) ;
}

int astar::compute_h_cost_Chebyshev( std::array<int,2> n ){
    int h = 0;

    int dx = abs( n[0] - goal_xy[0] );
    int dy = abs( n[1] - goal_xy[1] );

    h = MAX(dx, dy) *10;

    return int(h) ;
}



void astar::explore_one_ite(std::vector< std::array<int, 2>> active_nodes){
    for(std::vector<std::array<int, 2>>::iterator it = active_nodes.begin(); it != active_nodes.end(); ++it){
        explore_one_node(*it);
    }
}


void astar::search(){

    std::cout << "Looking for path " << std::endl;

    while (node_parent.count(goal_xy) == 0){
        explore_one_ite( find_min_cost_nodes() );
        if(FLAG_update_map_for_view){
            show_resize_img(&map_for_view, 10, 2, window_name);
        }
    }

    std::cout << "\nFind the goal" << std::endl;
}



void astar::get_path_no_return(){
    std::array<int, 2> get_path_curr_node = goal_xy;
    
    while (path.front() != start_xy)
    {
        path.push_front( get_path_curr_node);
        get_path_curr_node = node_parent[get_path_curr_node];
    }

    for(int pi =0; pi<path.size(); pi++){
        int row = path[pi][1];
        int col = path[pi][0];
        cv::Vec3b color = {0,255,0} ;
        map_for_view.at<cv::Vec3b>(row, col) = color; 
    }
    draw_start_goal_on_map();
    show_resize_img(&map_for_view, 10, 0, window_name);
}



void astar::print_path(){
    std::cout << "Find path:" << std::endl;
    for(auto p : path){
        std::cout << p[0] << " " << p[1] << std::endl;
    }
}


void astar::print_init_info(){

    std::cout << "\nastar start" 
    << "\n\nmap size:  \nw: " << map_width << " h: " << map_height
    << "\n\nStart location: \n" << start_xy[0] << " " << start_xy[1]
    << "\n\nGoal location : \n" << goal_xy[0] << " " << goal_xy[1]
    << "\n\nmotion model: \n" << motion_model[0][0] << "  " << motion_model[0][1]
    << "\n" << motion_model[1][0] << "  " << motion_model[1][1]
    << "\n" << motion_model[2][0] << "  " << motion_model[2][1]
    << "\n" << motion_model[3][0] << "  " << motion_model[3][1]
    << "\n" << motion_model[4][0] << "  " << motion_model[4][1]
    << "\n" << motion_model[5][0] << "  " << motion_model[5][1]
    << "\n" << motion_model[6][0] << "  " << motion_model[6][1]
    << "\n" << motion_model[7][0] << "  " << motion_model[7][1]

    << std::endl;

}




void astar::draw_start_goal_on_map(){
    cv::Vec3b color = {0,0,255} ;
    map_for_view.at<cv::Vec3b>(start_xy[1], start_xy[0]) = color;
    map_for_view.at<cv::Vec3b>(goal_xy[1], goal_xy[0]) = color;
}










