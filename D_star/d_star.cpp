
#include <iostream>
#include "d_star.h"
#include <string>
#include <stdexcept>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <limits>
#include <chrono>
#include <thread>


#include "utils/img_io.h"

D_star::D_star(int startnode[],
               int goalnode[],
               cv::Mat map)
{
    grid_map = map;
    map_for_view = map.clone();

    cv::threshold(grid_map.clone(), map_collision, 100, 255, 1);

    map_width = grid_map.size().width;
    map_height = grid_map.size().height;

    start_xy[0] = startnode[0];
    start_xy[1] = startnode[1];
    goal_xy[0] = goalnode[0];
    goal_xy[1] = goalnode[1];

    open_nodes.insert( goal_xy);
    all_nodes[goal_xy].g_cost = 0;

    draw_start_goal_on_map();

    curr_xy = goal_xy;

    build_motion_model();

    print_init_info();
}

D_star::~D_star()
{
}

void D_star::move_robot()
{
    
    map_for_view = grid_map.clone();
    std::array<int,2> robot_pose = path.back();
    path.pop_back();
    map_for_view.at<cv::Vec3b>(robot_pose[1],robot_pose[0]) = cv::Vec3b(0,0,255);

    std::array<int,2> robot_pose_next = path.back();

    if( check_collision_by_point(robot_pose_next[0], robot_pose_next[1]) )
    {
        // draw_all_nodes_on_map();
        draw_path_on_map();
        draw_start_goal_on_map();
        show_resize_img(&map_for_view, 10, 50, window_name);
    }
    else
    {
        std::deque< std::array<int, 2>> temp_path;
        temp_path.push_front(robot_pose);

        std::array<int,2> v_robot_pose = robot_pose;
        std::array<int,2> v_robot_pose_last;
        int v_step = 0;

        while (temp_path.front() != goal_xy)
        {
            v_robot_pose = temp_path.front();
            d_star_MODIFY_COST(v_robot_pose);
            d_star_PROCESS_STATE();

            std::array<int,2> new_parent = all_nodes[v_robot_pose].parent_grid;


            map_for_view.at<cv::Vec3b>( v_robot_pose[1],v_robot_pose[0]) = cv::Vec3b(0,255,0);
            show_resize_img(&map_for_view, 10, 20, window_name);


            if(new_parent != temp_path.front())
            {
                temp_path.push_front(new_parent);
            }
        }

        path.clear();
        path = temp_path;

        draw_path_on_map();
        draw_start_goal_on_map();
        show_resize_img(&map_for_view, 10, 1000, window_name);
        
    }

    
    
    
}


void D_star::build_motion_model()
{
    int xs[8] = {-1, 0, 1, 1, 1, 0, -1, -1};
    int ys[8] = {1, 1, 1, 0, -1, -1, -1, 0};
    int cost[8] = {14, 10, 14, 10, 14, 10, 14, 10};
    for (int i = 0; i < 8; i++)
    {
        motion_model[i][0] = xs[i];
        motion_model[i][1] = ys[i];
        motion_model[i][2] = cost[i];
    }
}

void D_star::d_star_INSERT(std::array<int, 2> grid, int hnew)
{
    // std::cout << "d_star_INSERT" << std::endl;

    if (open_nodes.count(grid) == 0 && close_nodes.count(grid) == 0)
    {
        open_nodes.insert(grid);
        all_nodes[grid].k = hnew;
    }

    else if (open_nodes.count(grid) == 1 && close_nodes.count(grid) == 0)
    {
        all_nodes[grid].k = MIN(all_nodes[grid].k, hnew);
    }

    else if (open_nodes.count(grid) == 0 && close_nodes.count(grid) == 1)
    {
        all_nodes[grid].k = MIN(all_nodes[grid].g_cost, hnew);
    }

    open_nodes.insert(grid);
    all_nodes[grid].g_cost = hnew;
}

int D_star::d_star_GET_KMIN()
{
    int kmin = std::numeric_limits<int>::max();
    for (auto on : open_nodes)
    {
        int k = all_nodes[on].k;
        if (k < kmin)
        {
            kmin = k;
        }
    }
    return kmin;
}

int D_star::d_star_MIN_STATE_and_GET_KMIN(std::vector<std::array<int, 2>> &Xs)
{
    
    int kmin = std::numeric_limits<int>::max();
    for (auto on : open_nodes)
    {
        int k = all_nodes[on].k;
        if (k < kmin)
        {
            kmin = k;
            Xs.clear();
            Xs.push_back(on);
        }
        else if (k == kmin)
        {
            Xs.push_back(on);
        }
    }
    return kmin;
}


void D_star::d_star_DELETE(std::array<int, 2> grid)
{
    open_nodes.erase(grid);
    close_nodes.insert(grid);
}


void D_star::d_star_PROCESS_STATE()
{
    // std::cout << "PROCESS_STATE" << std::endl;
    // std::cout << "open_nodes.size : " << open_nodes.size()  << std::endl;
    // std::cout << "close_nodes.size : " << close_nodes.size()  << std::endl;

    if (open_nodes.size() > 0)
    {
        int kold = 0;
        std::vector<std::array<int, 2>> Xs;
        kold = d_star_MIN_STATE_and_GET_KMIN(Xs);
        // std::cout << "kold = " << kold <<  std::endl;
        // std::cout << "Xs size: " << Xs.size() <<  std::endl;
        // std::cout << "Xs : " << Xs[0][0] << " " << Xs[0][1] <<  std::endl;

        int i = 0;

        for (auto curr : Xs)
        {
            d_star_DELETE(curr);

            i++;

            // First case
            if (kold < all_nodes[curr].g_cost)
            {
                nodeInfo best_nb;
                best_nb.g_cost = std::numeric_limits<int>::max();

                for (auto mm : motion_model)
                {
                    int x = curr[0] + mm[0];
                    int y = curr[1] + mm[1];
                    int edge_cost = mm[2];
                    if (check_collision_by_point(x, y) && check_valid_by_point(x, y))
                    {
                        std::array<int, 2> nb_grid = {x, y};
                        int h_Y = all_nodes[nb_grid].g_cost;

                        if(h_Y + edge_cost < best_nb.g_cost)
                        {
                            best_nb.parent_grid = nb_grid;
                            best_nb.g_cost = h_Y + edge_cost ;
                        }

                        if (h_Y <= kold + 6 && all_nodes[curr].g_cost > (h_Y + edge_cost))
                        {
                            all_nodes[curr].parent_grid = best_nb.parent_grid;
                            all_nodes[curr].g_cost = best_nb.g_cost;
                        }
                    }
                }
            }

            // Second case
            else if (kold == all_nodes[curr].g_cost)
            {
                for (auto mm : motion_model)
                {
                    int x = curr[0] + mm[0];
                    int y = curr[1] + mm[1];
                    int edge_cost = mm[2];
                    if (check_collision_by_point(x, y) && check_valid_by_point(x, y))
                    {
                        // std::cout << "nb_grid: "<< x << " " << y << "  ";// << std::endl;

                        std::array<int, 2> nb_grid = {x, y};
                        bool isNew = ! (open_nodes.count(nb_grid) || close_nodes.count(nb_grid) ) ;
                        bool second_term = (all_nodes[nb_grid].parent_grid == curr) && (all_nodes[nb_grid].g_cost != all_nodes[curr].g_cost + edge_cost);
                        bool third_term = (all_nodes[nb_grid].parent_grid != curr) && (all_nodes[nb_grid].g_cost > all_nodes[curr].g_cost + edge_cost);

                        if (isNew || second_term || third_term)
                        {
                            all_nodes[nb_grid].parent_grid = curr;
                            d_star_INSERT(nb_grid, all_nodes[curr].g_cost + edge_cost);
                        }
                    }
                }
            }

            // Third case
            else
            {
                for (auto mm : motion_model)
                {
                    int x = curr[0] + mm[0];
                    int y = curr[1] + mm[1];
                    int edge_cost = mm[2];
                    if (check_collision_by_point(x, y) && check_valid_by_point(x, y))
                    {
                        std::array<int, 2> nb_grid = {x, y};
                        bool isNew = !open_nodes.count(nb_grid);
                        bool second_term = (all_nodes[nb_grid].parent_grid == curr) && (all_nodes[nb_grid].g_cost != all_nodes[curr].g_cost + edge_cost);
                        if (isNew || second_term)
                        {
                            all_nodes[nb_grid].parent_grid = curr;
                            d_star_INSERT(nb_grid, all_nodes[curr].g_cost + edge_cost);
                        }
                        else
                        {
                            if ((all_nodes[nb_grid].parent_grid != curr) && (all_nodes[nb_grid].g_cost > all_nodes[curr].g_cost + edge_cost))
                            {
                                d_star_INSERT(curr, all_nodes[curr].g_cost);
                            }
                            else
                            {
                                bool term_1 = all_nodes[nb_grid].parent_grid != curr;
                                bool term_2 = all_nodes[curr].g_cost > all_nodes[nb_grid].g_cost + edge_cost;
                                bool term_3 = close_nodes.count(nb_grid) == 1 && open_nodes.count(nb_grid) == 0;
                                bool term_4 = all_nodes[nb_grid].g_cost > kold;
                                if (term_1 && term_2 && term_3 && term_4)
                                {
                                    d_star_INSERT(nb_grid, all_nodes[nb_grid].g_cost);
                                }
                            }
                        }
                    }
                }
            }


            // std::cout << "open:" << std::endl;
            // for(auto on : open_nodes)
            // {
            //     auto n = all_nodes[on];
            //     std::cout << on[0] << " " << on[1] << " h: " << n.g_cost << " k: " << n.k << std::endl;
            // }

            // std::cout << "close:" << std::endl;
            // for(auto on : close_nodes)
            // {
            //     auto n = all_nodes[on];
            //     std::cout << on[0] << " " << on[1] << " h: " << n.g_cost << " k: " << n.k << std::endl;
            // }

            // for(auto n : all_nodes)
            // {
            // //     map_for_view = grid_map.clone();
            // //     std::cout << "" << std::endl;
            //     // std::cout << n.first[0] << " " << n.first[1] << std::endl;
            //     std::cout << n.first[0] << " " << n.first[1] << " h: " << n.second.g_cost << " k: " << n.second.k << std::endl;
            // //     std::cout << n.second.parent_grid[0] << " " << n.second.parent_grid[1] << std::endl;
            // //     map_for_view.at<cv::Vec3b>( n.first[1],n.first[0]) = cv::Vec3b(0,255,255);
            // //     map_for_view.at<cv::Vec3b>( n.second.parent_grid[1] , n.second.parent_grid[0] ) = cv::Vec3b(0,155,255);
            // //     show_resize_img(&map_for_view, 10, 0, window_name);
            // }

            // draw_all_nodes_on_map();
            // show_resize_img(&map_for_view, 10, 2, window_name);


        }
    }
}


void D_star::d_star_MODIFY_COST(std::array<int,2> grid)
{
    int cval = std::numeric_limits<int>::max();

    if (open_nodes.count(grid) == 1 && close_nodes.count(grid) == 0)
    {
        all_nodes[grid].k = MIN(all_nodes[grid].k, cval);
    }

    else if (open_nodes.count(grid) == 0 && close_nodes.count(grid) == 1)
    {
        all_nodes[grid].k = MIN(all_nodes[grid].g_cost, cval);
        close_nodes.erase(grid);
        open_nodes.insert(grid);
    }

    all_nodes[grid].g_cost = (cval-1)/2;

    // std::cout << "MODIFY_COST  " << grid[0] << " " << grid[1] << std::endl;
    // std::cout << "open: " << open_nodes.size() << std::endl;
    // std::cout << "close: " <<  close_nodes.size() << std::endl;

    show_resize_img(&map_for_view, 10, 50, window_name);
}



void D_star::update_obs_map( std::vector<int> new_obs)
{
    int x1 = new_obs[0];
    int y1 = new_obs[1];
    
    int x2 = new_obs[2];
    int y2 = new_obs[3];

    for(int x=x1; x<x2; x++)
    {
        for(int y=y1; y<y2; y++)
        {
            std::array<int,2> grid = {x,y} ;
            grid_map.at<cv::Vec3b>(y,x) = cv::Vec3b( 0,0,0 );
            if (open_nodes.count(grid) == 1 && close_nodes.count(grid) == 0)
            {
                open_nodes.erase(grid);
            }

            else if (open_nodes.count(grid) == 0 && close_nodes.count(grid) == 1)
            {
                close_nodes.erase(grid);
            }
        }
    }

    cv::threshold(grid_map.clone(), map_collision, 100, 255, 1);

}


void D_star::search()
{

    std::cout << "Looking for path " << std::endl;

    int i = 0;

    while (close_nodes.count(start_xy) == 0)
    {
        std::cout << "\n\n" << i << "-th " << std::endl;

        d_star_PROCESS_STATE();

        if (FLAG_update_map_for_view)
        {
            draw_all_nodes_on_map();
            show_resize_img(&map_for_view, 10, 2, window_name);
        }

        i++;
    }

    std::cout << "\nFind the goal" << std::endl;

    std::cout << "open: " << open_nodes.size() << std::endl;

    std::cout << "close: " <<  close_nodes.size() << std::endl;

    show_resize_img(&map_for_view, 10, 1000, window_name);
}



void D_star::get_path()
{
    std::cout << "\nget_path" << std::endl;
    std::array<int, 2> get_path_curr_node = start_xy;
    
    while ( get_path_curr_node != goal_xy )
    {
        std::cout << get_path_curr_node[0] << " " << get_path_curr_node[1] << std::endl;
        cv::Vec3b color = {0, 0, 255};
        map_for_view.at<cv::Vec3b>(get_path_curr_node[1], get_path_curr_node[0]) = color;
        show_resize_img(&map_for_view, 10, 2, window_name);

        path.push_front(get_path_curr_node);
        get_path_curr_node = all_nodes[get_path_curr_node].parent_grid;
        // std::this_thread::sleep_for( std::chrono::milliseconds(300));
    }

    draw_start_goal_on_map();
    show_resize_img(&map_for_view, 10, 1000, window_name);
}


void D_star::draw_path_on_map()
{
    for(auto pn : path)
    {
        map_for_view.at<cv::Vec3b>(pn[1],pn[0]) = cv::Vec3b(0,100,200);
    }
}


void D_star::draw_all_nodes_on_map()
{
    map_for_view = grid_map.clone();

    cv::Vec3b color = {0,255,0};
    for(auto n : open_nodes)
    {
        map_for_view.at<cv::Vec3b>(n[1],n[0]) = color;
    }

    color = {255,55,0};
    for(auto n : close_nodes)
    {
        map_for_view.at<cv::Vec3b>(n[1],n[0]) = color;
    }

    draw_start_goal_on_map();
}


bool D_star::check_collision_by_point(int x, int y)
{
    bool out;
    if (map_collision.at<cv::Vec3b>(y, x)[0] != 0)
        out = false;
    else
        out = true;
    return out;
}

bool D_star::check_valid_by_point(int x, int y)
{
    if (0 <= x && x < map_width && 0 <= y && y < map_height)
    {
        return true;
    }
    else
    {
        return false;
    }
}




void D_star::print_init_info()
{

    std::cout << "\nD_star start"
              << "\n\nmap size:  \nw: " << map_width << " h: " << map_height
              << "\n\nStart location: \n"
              << start_xy[0] << " " << start_xy[1]
              << "\n\nGoal location : \n"
              << goal_xy[0] << " " << goal_xy[1]
              << "\n\nmotion model: \n"
              << motion_model[0][0] << "  " << motion_model[0][1]
              << "\n"
              << motion_model[1][0] << "  " << motion_model[1][1]
              << "\n"
              << motion_model[2][0] << "  " << motion_model[2][1]
              << "\n"
              << motion_model[3][0] << "  " << motion_model[3][1]
              << "\n"
              << motion_model[4][0] << "  " << motion_model[4][1]
              << "\n"
              << motion_model[5][0] << "  " << motion_model[5][1]
              << "\n"
              << motion_model[6][0] << "  " << motion_model[6][1]
              << "\n"
              << motion_model[7][0] << "  " << motion_model[7][1]

              << std::endl;
}

void D_star::draw_start_goal_on_map()
{
    cv::Vec3b color = {0, 0, 255};
    map_for_view.at<cv::Vec3b>(start_xy[1], start_xy[0]) = color;
    map_for_view.at<cv::Vec3b>(goal_xy[1], goal_xy[0]) = color;
}
