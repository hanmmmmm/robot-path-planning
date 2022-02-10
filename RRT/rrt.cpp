
#include <iostream>
#include "rrt.h"
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


RRT::RRT(std::array<int, 2> &startnode, std::array<int, 2> &goalnode, cv::Mat map)
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

RRT::~RRT()
{
}

bool RRT::check_collision_by_point(int x, int y)
{
    bool out;
    if (map_collision.at<cv::Vec3b>(y, x)[0] == 0)
        out = false;
    else
        out = true;
    return out;
}

bool RRT::check_collision_by_line(std::array<int, 2> pose1, std::array<int, 2> pose2, int num_step)
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

void RRT::test_collision_func()
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

void RRT::trauncate_line(std::array<int, 2> &root, std::array<int, 2> &end, int max_length)
{
    int xr = root[0];
    int yr = root[1];
    int xe = end[0];
    int ye = end[1];
    int dx = xe - xr;
    int dy = ye - yr;
    float ratio = float(max_length * max_length) / float(dx * dx + dy * dy);
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

void RRT::sample_a_point(int ite)
{
    std::array<int, 2> curr_xy;
    int remainder = std::div(ite, 3).rem;

    int x = std::rand() % fine_map_width + 1;
    int y = std::rand() % fine_map_height + 1;
    curr_xy = {x, y};

    if (remainder == 0)
    {
        curr_xy = goal_pose;
    }

    std::cout << "curr_xy  " << curr_xy[0] << " " << curr_xy[1] << std::endl;

    if (check_collision_by_point(curr_xy[0], curr_xy[1]) && all_nodes.count(curr_xy) == 0 && curr_xy != start_pose)
    {
        int num_valid_connection = 0;
        nodeInfo temp_info;

        std::array<int, 2> closest_node;
        int min_cost = std::numeric_limits<int>::max();
        for (auto n : all_nodes)
        {
            int cost = compute_Euclidean_dist(curr_xy, n.first);
            if (cost < min_cost)
            {
                min_cost = cost;
                closest_node = n.first;
            }
        }

        trauncate_line(closest_node, curr_xy, k_max_step_length);

        if (all_nodes.count(curr_xy) == 0 && check_collision_by_line(curr_xy, closest_node, 400))
        {
            nodeInfo temp;
            temp.parent = closest_node;
            temp.cost = all_nodes[closest_node].cost + min_cost;
            all_nodes[curr_xy] = temp;

            if (curr_xy == goal_pose)
            {
                FLAG_reach_goal = true;
            }

            if (FLAG_update_map_for_view)
            {
                cv::line(map_for_view, cv::Point(curr_xy[0], curr_xy[1]), cv::Point(closest_node[0], closest_node[1]), cv::Scalar(220, 0, 0), 2);
                show_resize_img(&map_for_view, 1.0, 2, k_window_name);
            }
        }
    }
}

void RRT::search()
{
    std::cout << "Looking for path " << std::endl;

    int i = 0;

    show_resize_img(&map_for_view, 1.0, 0, k_window_name);

    while (!FLAG_reach_goal)
    {
        sample_a_point(i);
        i++;

        if (FLAG_update_map_for_view)
            show_resize_img(&map_for_view, 1.0, 2, k_window_name);
    }

    std::cout << "\nFound the goal" << std::endl;
    show_resize_img(&map_for_view, 1.0, 0, k_window_name);
}

void RRT::get_path()
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

    show_resize_img(&map_for_view, 1.0, 0, k_window_name);
}

// void RRT::get_path_dfs_recursive()
// {
//     std::array<int, 2> curr = start_pose;

//     dfs_recursive( curr, goal_pose );

// }

// void RRT::dfs_recursive(std::array<int,2> curr, std::array<int,2> goal)
// {
//     for(auto nb : all_nodes[curr].connected_nodes)
//     {
//         if( nb == goal)
//         {
//             std::cout << "\nnum of nodes " << all_nodes.size() << std::endl;
//             std::cout << "\nFound a path " << all_path.size()+1 << std::endl;

//             std::vector< std::array<int,2> > temp_buff;
//             for(auto node : path_buff )
//             {
//                 temp_buff.push_back(node);
//                 std::cout << "[ " << node[0] << " " << node[1] << " ] " ;

//             }
//             std::cout << std::endl;
//             all_path.push_back( temp_buff);

//         }
//         else if ( !( std::find( path_buff.begin(), path_buff.end(), nb) != path_buff.end() ) )
//         {
//             path_buff.push_back(nb);
//             dfs_recursive( nb, goal );
//             path_buff.pop_back();
//         }

//     }
// }

// void RRT::get_path()
// {
//     std::deque<std::array<int, 2>> q;
//     std::deque<std::array<int, 2>> q_closed;

//     std::array<int, 2> curr = start_pose;
//     std::cout << "\nTraverse start" << std::endl;
//     q.push_front(curr);

//     while (curr != goal_pose)
//     {
//         curr = q.front();

//         for (auto n : all_nodes[curr].connected_nodes)
//         {
//             bool new_point = true;
//             for (auto i = q.begin(); i < q.end(); i++)
//             {
//                 if (*i == n)
//                 {
//                     new_point = false;
//                     break;
//                 }
//             }
//             for (auto i = q_closed.begin(); i < q_closed.end(); i++)
//             {
//                 if (*i == n)
//                 {
//                     new_point = false;
//                     break;
//                 }
//             }
//             if (new_point)
//             {
//                 cv::line(map_for_view, cv::Point(curr[0], curr[1]), cv::Point(n[0], n[1]), cv::Scalar(0, 0, 200), 2);
//                 show_resize_img(&map_for_view, 1.0, 2, k_window_name);

//                 q.push_back(n);
//                 all_nodes[n].parent = curr;
//             }
//         }
//         q_closed.push_back(curr);
//         q.pop_front();
//     }

//     std::cout << "\nTraverse done" << std::endl;

//     cv::Scalar line_color(0, 255, 55);
//     std::array<int, 2> get_path_curr_node = goal_pose;
//     path.push_front(goal_pose);
//     while (path.front() != start_pose)
//     {
//         std::array<int,2> prev_point = path[0];
//         std::cout << prev_point[0] <<  " " << prev_point[1] << std::endl;
//         get_path_curr_node = all_nodes[ path[0] ].parent;
//         path.push_front( get_path_curr_node );
//         std::array<int,2> curr_point = path[0];

//         cv::Point p1(prev_point[0], prev_point[1]);
//         cv::Point p2(curr_point[0], curr_point[1]);

//         cv::line(map_for_view, p1, p2, line_color, 10);
//     }
//     std::cout << "\nDraw path done" << std::endl;

//     show_resize_img(&map_for_view, 1.0, 0, k_window_name);
// }
