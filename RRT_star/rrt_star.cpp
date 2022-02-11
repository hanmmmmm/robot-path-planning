
#include <iostream>
#include "rrt_star.h"
#include <string>
#include <stdexcept>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <limits>
#include <math.h>
#include <random>
#include <stack>
#include <algorithm>

#include "utils/img_io.h"
#include "utils/math_tools.h"


RRT_star::RRT_star(std::array<int, 2> &startnode, std::array<int, 2> &goalnode, cv::Mat map)
{
    // prepare the maps

    cv::Mat map_grid = map;

    cv::resize(map, map_fine, cv::Size(), k_fine_ratio, k_fine_ratio, cv::INTER_NEAREST);
    map_for_view = map_fine.clone();

    cv::threshold(map_fine.clone(), map_collision, 100, 255, 1);

    cv::Mat erode_element = cv::getStructuringElement(0, cv::Size(2 * k_map_inflation_size + 1, 2 * k_map_inflation_size + 1), cv::Point(k_map_inflation_size, k_map_inflation_size));
    cv::erode(map_fine, map_collision, erode_element);

    fine_map_width = map_fine.size().width;
    fine_map_height = map_fine.size().height;

    // start_pose[0] = startnode[0];
    // start_pose[1] = startnode[1];
    start_pose = startnode;

    all_nodes[start_pose].cost = 0;

    // goal_pose[0] = goalnode[0];
    // goal_pose[1] = goalnode[1];
    goal_pose = goalnode;

    std::srand(time(NULL));

    draw_start_goal_on_map();

    print_init_info();
}

RRT_star::~RRT_star()
{
}

bool RRT_star::check_collision_by_point(int x, int y)
{
    bool out;
    if (map_collision.at<cv::Vec3b>(y, x)[0] == 0)
        out = false;
    else
        out = true;
    return out;
}



bool RRT_star::check_collision_by_line(std::array<int, 2> pose1, std::array<int, 2> pose2, int num_step)
{
    float x1 = pose1[0];
    float y1 = pose1[1];

    float x2 = pose2[0];
    float y2 = pose2[1];

    float dx = (x2 - x1) / num_step;
    float dy = (y2 - y1) / num_step;

    for (int i = 0; i < num_step; i++)
    {
        int x = x1 + i * dx;
        int y = y1 + i * dy;
        if (!check_collision_by_point(x, y))
        {
            return false;
        }
    }
    return true;
}



void RRT_star::test_collision_func()
{
    while (true)
    {
        map_for_view = map_fine.clone();

        int x = std::rand() % fine_map_width + 1;
        int y = std::rand() % fine_map_height + 1;
        std::array<int, 2> pose1{x, y};

        cv::circle(map_for_view, cv::Point(x, y), 5, cv::Scalar(250, 0, 20), -1);
        x = std::rand() % fine_map_width + 1;
        y = std::rand() % fine_map_height + 1;
        std::array<int, 2> pose2{x, y};

        cv::circle(map_for_view, cv::Point(x, y), 5, cv::Scalar(250, 0, 20), -1);
        bool col = check_collision_by_line(pose1, pose2, 10);
        std::cout << "collision " << col << std::endl;
        show_resize_img(&map_for_view, 1.0, 0, k_window_name);
    }
}



void RRT_star::trauncate_line(std::array<int, 2> &root, std::array<int, 2> &end, int max_length)
{
    int xr = root[0];
    int yr = root[1];
    int xe = end[0];
    int ye = end[1];
    int dx = xe - xr;
    int dy = ye - yr;
    float ratio = float( max_length ) / sqrt(dx * dx + dy * dy);
    std::cout << "ratio " << ratio << std::endl;
    if (ratio < 1.0)
    {
        end[0] = xr + ratio * dx;
        end[1] = yr + ratio * dy;
    }
    else
    {
        end[0] = xe;
        end[1] = ye;
    }
}



void RRT_star::sample_a_point(int ite, bool use_goal)
{
    std::array<int, 2> curr_xy;
    int remainder = std::div(ite, 3).rem;

    int x = std::rand() % fine_map_width + 1;
    int y = std::rand() % fine_map_height + 1;
    curr_xy = {x, y};

    if( ! use_goal && curr_xy == goal_pose)
    {
        return;
    }

    if (remainder == 0  && use_goal)
    {
        curr_xy = goal_pose;
    }

    // std::cout << "curr_xy  " << curr_xy[0] << " " << curr_xy[1] << std::endl;

    if (check_collision_by_point(curr_xy[0], curr_xy[1]) && all_nodes.count(curr_xy) == 0 && curr_xy != start_pose)
    {
        int num_valid_connection = 0;
        nodeInfo temp_info;

        std::array<int, 2> closest_node;
        int min_cost = std::numeric_limits<int>::max();
        for (auto n : all_nodes)
        {
            int edge_cost = compute_Euclidean_dist(curr_xy, n.first);
            // int total_cost = edge_cost + n.second.cost;
            if (edge_cost < min_cost ) // &&  check_collision_by_line(curr_xy, n.first, 400))
            {
                min_cost = edge_cost;
                closest_node = n.first;
            }
        }

        std::cout << "curr_xy before" << curr_xy[0] << " " << curr_xy[1] << std::endl;
        trauncate_line(closest_node, curr_xy, k_max_step_length);
        if( curr_xy == goal_pose  && FLAG_reach_goal)
        {
            return;
        }
        std::cout << "curr_xy after" << curr_xy[0] << " " << curr_xy[1] << std::endl;

        std::vector< std::array<int,2> > near_nodes ;

        if (all_nodes.count(curr_xy) == 0 )
        {
            std::array<int, 2> closest_node_new;
            int min_cost = std::numeric_limits<int>::max();
            bool connected = false;
            for (auto n : all_nodes)
            {
                int edge_cost = compute_Euclidean_dist(curr_xy, n.first);
                int total_cost = edge_cost + n.second.cost;

                if(edge_cost < k_max_step_length){
                    if (total_cost < min_cost  &&  check_collision_by_line(curr_xy, n.first, 400) && n.first != goal_pose)
                    {
                        min_cost = total_cost;
                        closest_node_new = n.first;
                        connected = true;
                    }
                }
                
                if(edge_cost < k_near_node_range )
                {
                    near_nodes.push_back( n.first );
                }

                
            }

            if( !connected )
            {
                return;
            }

            all_nodes[curr_xy].parent = closest_node_new;
            all_nodes[curr_xy].cost   = min_cost;
            
            // std::cout << "curr_xy " << curr_xy[0] << " " << curr_xy[1] << std::endl;
            // std::cout << "closest_node_new " << closest_node_new[0] << " " << closest_node_new[1] << std::endl;
            // std::cout << "check_collision " << check_collision_by_line(curr_xy, closest_node_new, 400)  << std::endl;
            // std::cout << "min_cost " << min_cost  << std::endl;


            int curr_node_cost = all_nodes[curr_xy].cost;
            for(auto nn : near_nodes)
            {
                int edge_cost = compute_Euclidean_dist( nn , curr_xy);
                int total_cost = curr_node_cost + edge_cost;
                if(total_cost < all_nodes[nn].cost  && nn!= closest_node && check_collision_by_line(nn, curr_xy, 200))
                {
                    all_nodes[nn].parent = curr_xy;
                    all_nodes[nn].cost   = total_cost;
                }
            }

            if (curr_xy == goal_pose)
            {
                FLAG_reach_goal = true;
            }

            if (FLAG_update_map_for_view)
            {
                cv::line(map_for_view, cv::Point(curr_xy[0], curr_xy[1]), cv::Point(closest_node_new[0], closest_node_new[1]), cv::Scalar(220, 0, 0), 2);
                show_resize_img(&map_for_view, 1.0, 2, k_window_name);
            }

        }


    }
}



void RRT_star::search()
{
    std::cout << "Looking for path " << std::endl;

    int i = 0;

    show_resize_img(&map_for_view, 1.0, 50, k_window_name);

    while (!FLAG_reach_goal && i < k_max_num_sample)
    {
        sample_a_point(i, true);
        i++;

        if (FLAG_update_map_for_view)
            show_resize_img(&map_for_view, 1.0, 2, k_window_name);
    }

    std::cout << "\nFound the goal" << std::endl;

    while (FLAG_reach_goal && i < k_max_num_sample)
    {
        sample_a_point(i, false );
        i++;

        if (FLAG_update_map_for_view)
            show_resize_img(&map_for_view, 1.0, 2, k_window_name);
    }

    show_resize_img(&map_for_view, 1.0, 10, k_window_name);
}



void RRT_star::get_path()
{
    std::cout << "\nExtracting path" << std::endl;

    std::array<int, 2> get_path_curr;
    get_path_curr = goal_pose;

    while (get_path_curr != start_pose)
    {
        std::cout << get_path_curr[0] << "  " << get_path_curr[1] << std::endl;

        cv::Point p1(get_path_curr[0], get_path_curr[1]);
        get_path_curr = all_nodes[get_path_curr].parent;
        cv::Point p2(get_path_curr[0], get_path_curr[1]);

        cv::line(map_for_view, p1, p2, cv::Scalar(0, 255, 0), 5);
        show_resize_img(&map_for_view, 1.0, 2, k_window_name);
    }

    show_resize_img(&map_for_view, 1.0, 2000, k_window_name);
}


