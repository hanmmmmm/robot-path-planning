
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
#include "utils/bresenham.h"
#include "utils/math_tools.h"



astar::astar(int startnode[], int goalnode[], float s_angle, float g_angle, cv::Mat map)
{
    // prepare the maps 

    map_grid = map;

    cv::resize(map, map_fine, cv::Size(), fine_ratio, fine_ratio, cv::INTER_NEAREST);
    map_for_view = map_fine.clone();

    cv::threshold(map_fine.clone(), map_collision, 100, 255, 1);

    cv::Mat erode_element = cv::getStructuringElement(0, cv::Size( 2*map_inflation_size+1, 2*map_inflation_size+1), cv::Point(map_inflation_size,map_inflation_size));
    cv::erode(map_fine, map_fine, erode_element);
    // cv::imshow("map fine", map_collision);

    fine_map_width = map_fine.size().width;
    fine_map_height = map_fine.size().height;

    grid_map_width = map_grid.size().width;
    grid_map_height = map_grid.size().height;

    start_pose[0] = startnode[0];
    start_pose[1] = startnode[1];
    start_pose[2] = s_angle;

    start_grid = fine_to_grid(start_pose, fine_ratio, angle_resolution);

    grid_fine_pose[start_grid] = start_pose;
    grid_parent[start_grid] = start_grid;
    grid_status[start_grid] = {0, 0, 1, 1};

    goal_pose[0] = goalnode[0];
    goal_pose[1] = goalnode[1];
    goal_pose[2] = g_angle;

    goal_grid = fine_to_grid(goal_pose, fine_ratio, angle_resolution);

    draw_start_goal_on_map();

    build_motion_model();

    print_init_info();
}




astar::~astar()
{
}




void astar::build_motion_model()
{

    for (int i = 0; i < turning_raius.size(); i++)
        turning_angle[i] = step_length / turning_raius[i];

    float raius = turning_raius[1];
    float angle = turning_angle[1];
    std::cout << "radius: " << raius << std::endl;
    std::cout << "angle: " << angle << std::endl;

    float unit_dx = raius * sin(angle);
    float unit_dy = raius * (cos(angle) - 1);

    std::vector<float> temp = {unit_dx, -1 * unit_dy, angle, 0};
    motion_model.push_back(temp);
    temp = {step_length, 0, 0, 0};
    motion_model.push_back(temp);
    temp = {unit_dx, unit_dy, float(M_PI * 2 - angle), 0};
    motion_model.push_back(temp);

    temp = {-1 * unit_dx, unit_dy, angle, 1};
    motion_model.push_back(temp);
    temp = {-1 * step_length, 0, float(M_PI * 2 - angle), 1};
    motion_model.push_back(temp);
    temp = {-1 * unit_dx, -1 * unit_dy, angle, 1};
    motion_model.push_back(temp);
}




void astar::explore_one_node(std::array<float, 3> curr_pose, std::array<int, 3> curr_grid)
{
    grid_status[curr_grid][2] = 2;
    float curr_theta = curr_pose[2];
    float cos_theta = cos(curr_theta);
    float sin_theta = sin(curr_theta);

    int count = 0;
    for (auto mm : motion_model)
    {
        float dx = cos_theta * mm[0] + sin_theta * mm[1];
        float dy = sin_theta * mm[0] + cos_theta * mm[1];
        float nb_x = curr_pose[0] + dx;
        float nb_y = curr_pose[1] + dy;

        if (0 <= nb_x && nb_x < fine_map_width && 0 <= nb_y && nb_y < fine_map_height)
        {
            if (map_fine.at<cv::Vec3b>(int(nb_y), int(nb_x))[0] == 255)
            { // if not obstacle

                int edge_cost = step_length;
                int is_reversing = int(mm[3]);
                if (is_reversing)
                    edge_cost *= 1.05;

                int nb_steer = count;
                int parent_steer = grid_status[curr_grid][3];

                float nb_a = curr_theta + mm[2]; // angle change in this step
                nb_a = rectify_angle_rad(nb_a);

                std::array<float, 3> nb_pose = {nb_x, nb_y, nb_a};
                std::array<int, 3> nb_grid = fine_to_grid(nb_pose, fine_ratio, angle_resolution);

                if (grid_parent.count(nb_grid) == 0)
                {
                    grid_fine_pose[nb_grid] = nb_pose;
                    grid_parent[nb_grid] = curr_grid;
                    grid_status[nb_grid][2] = 1;
                    grid_status[nb_grid][3] = count;
                    int gcost = grid_status[curr_grid][0] + edge_cost;
                    grid_status[nb_grid][0] = gcost;

                    int hcost = compute_h_cost_Euclidean(nb_pose, goal_pose);

                    hcost = improve_hcost(nb_pose, hcost, obstacle_scan_range, nb_steer==parent_steer);

                    grid_status[nb_grid][1] = gcost + hcost;

                    if (compute_h_cost_Euclidean(nb_pose, goal_pose) < 12)
                    {
                        if (abs(nb_a - goal_pose[2]) < 0.2)
                        {
                            FLAG_reach_goal = true;
                            close_goal_angle = nb_a;
                            close_goal_pose = nb_pose;
                            grid_parent[goal_grid] = curr_grid;
                            grid_fine_pose[goal_grid] = nb_pose;
                            grid_status[goal_grid][2] = 2;
                        }
                    }

                    if (FLAG_update_map_for_view)
                    {
                        if(map_grid.at<cv::Vec3b>(nb_grid[1], nb_grid[0])[0] != 0 ){
                            cv::Point p1(curr_pose[0], curr_pose[1]);
                            cv::Point p2(nb_pose[0], nb_pose[1]);
                            cv::Scalar line_color(0, 0, 200);
                            // cv::circle(map_for_view, p1, 2, cv::Scalar(255,0,0),-1);
                            draw_arc(curr_pose, nb_pose, turning_raius[1],turning_angle[1],nb_steer, 1, cv::Scalar(20,200,0));
                            // show_resize_img(&map_for_view, 1.6, 0, window_name);
                        }
                    }
                }
                else{
                    if(grid_status[nb_grid][2] == 1){
                        int old_cost = grid_status[nb_grid][1];
                        int gcost = grid_status[curr_grid][0] + edge_cost;
                        grid_status[nb_grid][0] = gcost;

                        int hcost = compute_h_cost_Euclidean(nb_pose, goal_pose);

                        hcost = improve_hcost(nb_pose, hcost, obstacle_scan_range, nb_steer==parent_steer);

                        int fcost = gcost + hcost;
                        
                        if(fcost < old_cost){
                            grid_fine_pose[nb_grid] = nb_pose;
                            grid_parent[nb_grid] = curr_grid;
                            grid_status[nb_grid][2] = 1;
                            grid_status[nb_grid][3] = count;
                            grid_status[nb_grid][0] = gcost;
                            grid_status[nb_grid][1] = fcost;
                        }
                    }
                }
            }
            else{
            }
        }
        count++;
    }
}





std::vector<std::array<int, 3>> astar::find_min_cost_nodes()
{
    // std::cout << "find_min_cost_nodes" << std::endl;
    std::vector<std::array<int, 3>> out;
    int min_cost = std::numeric_limits<int>::max();
    
    for (auto n : grid_status) // if a node exist in node_parent
    {
        std::array<int, 3> node = n.first;
        if (n.second[2] == 1)  // if state == open
        {
            int cost = n.second[1];
            if (cost < min_cost)
            {
                min_cost = cost;
                out.clear();
                out.push_back(node);
            }
        }
    }
    return out;
}




void astar::explore_one_ite(std::vector<std::array<int, 3>> active_nodes)
{
    for (std::vector<std::array<int, 3>>::iterator it = active_nodes.begin(); it != active_nodes.end(); ++it)
    {
        auto fine_pose = grid_fine_pose[*it];
        explore_one_node(fine_pose, *it);
    }
}





void astar::search()
{
    std::cout << "Looking for path " << std::endl;

    while (!FLAG_reach_goal)
    {
        explore_one_ite(find_min_cost_nodes());

        if (FLAG_update_map_for_view)
            show_resize_img(&map_for_view, 1.0, 2, window_name);
        
    }

    std::cout << "\nFound the goal" << std::endl;
}




void astar::get_path()
{
    std::array<int, 3> get_path_curr_node = fine_to_grid(close_goal_pose, fine_ratio, angle_resolution);
    std::cout << "get_path" << std::endl;
    cv::Scalar line_color(0, 0, 255);
    // path.push_front(goal_grid);
    cv::line(map_for_view, cv::Point(goal_pose[0],goal_pose[1]), cv::Point(close_goal_pose[0],close_goal_pose[1]), line_color, 3);
    cv::circle(map_for_view, cv::Point(close_goal_pose[0],close_goal_pose[1]), 4, cv::Scalar(255, 0, 0), -1, 8, 0);
    while (path.front() != start_grid)
    {
        path.push_front(get_path_curr_node);
        std::array<float, 3> prev_point = grid_fine_pose[get_path_curr_node];
        get_path_curr_node = grid_parent[get_path_curr_node];
        std::array<float, 3> curr_point = grid_fine_pose[get_path_curr_node];

        cv::Point p1(prev_point[0], prev_point[1]);
        cv::Point p2(curr_point[0], curr_point[1]);

        cv::line(map_for_view, p1, p2, line_color, 3);
        // draw_arc(grid_fine_pose[get_path_curr_node], grid_fine_pose[ path.front()], turning_raius[1], turning_angle[1], motion, 2, cv::Scalar(0,50,255));
        cv::circle(map_for_view, p2, 4, cv::Scalar(255, 0, 0), -1, 8, 0);
    }
    // cv::line(map_for_view, p1, p2, line_color, 3);
    // cv::circle(map_for_view, p2, 4, cv::Scalar(255, 0, 0), -1, 8, 0);

    draw_start_goal_on_map();
    show_resize_img(&map_for_view, 1.0, 0, window_name);
}


