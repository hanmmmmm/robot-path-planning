
#include <iostream>
#include "dfs.h"
#include <string>
#include <stdexcept>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
// #include "utils/img_io.cpp"
// #include "utils/img_show.cpp"
#include "utils/img_io.h"

dfs::dfs(int startnode[],
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

    // if(mapCv.at<cv::Vec3b>(start_xy[1], start_xy[0]) != 0 )

    q.push(start_xy);
    parent_nodes[start_xy] = start_xy;

    build_motion_model();

    print_init_info();
}

dfs::~dfs()
{
}



void dfs::build_motion_model(){
    int xs[8] = {-1,0,1,1,1,0,-1,-1};
    int ys[8] = {1,1,1,0,-1,-1,-1,0};
    for(int i = 0; i<8; i++){
        motion_model[i][0] = xs[i];
        motion_model[i][1] = ys[i];
    }
}




void dfs::explore_one_node(){
    
    // std::cout << " q size: " << q.size() << std::endl;
    curr_xy = q.top();
    q.pop();
    // std::cout << "curr_xy  " << curr_xy[0] << " " << curr_xy[1] << std::endl;
    
    int num_added_node = 0;
    for(auto mm : motion_model){
        int n_x = curr_xy[0] + mm[0];
        int n_y = curr_xy[1] + mm[1];

        // std::cout << n_x << "  " << n_y << std::endl;

        if(0<= n_x && n_x < map_width && 0<= n_y && n_y < map_height){
            if(mapCv.at<cv::Vec3b>(n_y, n_x)[0] != 0){ // if not obstacle
                std::array<int,2> n_node = {n_x, n_y};
                if(parent_nodes.count(n_node) == 0){
                    parent_nodes[n_node] = curr_xy;
                    num_added_node ++;
                    q.push(n_node);

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

    // if(num_added_node == 0){
    //     throw std::invalid_argument("this node is trapped");
    // }
}



void dfs::search(){
    std::cout << "Looking for path " << std::endl;
    while (parent_nodes.count(goal_xy) == 0){
        explore_one_node();
        if(FLAG_update_map_for_view){
            show_resize_img(&map_for_view, 10, 2, "map");
        }
    }
    std::cout << "\nFind the goal" << std::endl;
}


std::deque< std::array<int, 2>> dfs::get_path(){
    get_path_no_return();
    return path;
}



void dfs::get_path_no_return(){
    std::array<int, 2> get_path_curr_node = goal_xy;
    std::deque< std::array<int, 2>> path;
    while (path.front() != start_xy)
    {
        path.push_front( get_path_curr_node);
        get_path_curr_node = parent_nodes[get_path_curr_node];
    }

    for(int pi =0; pi<path.size(); pi++){
        int row = path[pi][1];
        int col = path[pi][0];
        cv::Vec3b color = {0,255,0} ;
        map_for_view.at<cv::Vec3b>(row, col) = color; 
    }
    draw_start_goal_on_map();
    show_resize_img(&map_for_view, 10, 0, "map");
}













void dfs::print_init_info(){

    std::cout << "\ndfs start" 
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


void dfs::print_path(){
    std::cout << "Find path:" << std::endl;
    for(auto p : path){
        std::cout << p[0] << " " << p[1] << std::endl;
    }
}





void dfs::draw_start_goal_on_map(){
    cv::Vec3b color = {0,0,255} ;
    map_for_view.at<cv::Vec3b>(start_xy[1], start_xy[0]) = color;
    map_for_view.at<cv::Vec3b>(goal_xy[1], goal_xy[0]) = color;
}












