#include "../rrt.h"



void RRT::print_path()
{
    std::cout << "Find path:" << std::endl;
    for (auto p : path)
    {
        std::cout << p[0] << " " << p[1] << std::endl;
    }
}




void RRT::print_init_info()
{

    std::cout << "\nRRT start"
              << "\n\nfine map size:  \nw: " << fine_map_width << " h: " << fine_map_height
            //   << "\n\ngrid map size:  \nw: " << grid_map_width << " h: " << grid_map_height

              << "\n\nStart location: \n"
              << start_pose[0] << " " << start_pose[1] << " " << start_pose[2]
            //   << "\n\nStart grid    : \n"
            //   << start_grid[0] << " " << start_grid[1] << " " << start_grid[2]
              << "\n\nGoal location : \n"
              << goal_pose[0] << " " << goal_pose[1] << " " << goal_pose[2]
            //   << "\n\nGoal grid     : \n"
            //   << goal_grid[0] << " " << goal_grid[1] << " " << goal_grid[2]

            //   << "\n\nmotion model: \n"
            //   << motion_model[0][0] << "  " << motion_model[0][1] << "  " << motion_model[0][2] << "  " << motion_model[0][3]
            //   << "\n"
            //   << motion_model[1][0] << "  " << motion_model[1][1] << "  " << motion_model[1][2] << "  " << motion_model[1][3]
            //   << "\n"
            //   << motion_model[2][0] << "  " << motion_model[2][1] << "  " << motion_model[2][2] << "  " << motion_model[2][3]
            //   << "\n"
            //   << motion_model[3][0] << "  " << motion_model[3][1] << "  " << motion_model[3][2] << "  " << motion_model[3][3]
            //   << "\n"
            //   << motion_model[4][0] << "  " << motion_model[4][1] << "  " << motion_model[4][2] << "  " << motion_model[4][3]
            //   << "\n"
            //   << motion_model[5][0] << "  " << motion_model[5][1] << "  " << motion_model[5][2] << "  " << motion_model[5][3]
              // << "\n" << motion_model[6][0] << "  " << motion_model[6][1]
              // << "\n" << motion_model[7][0] << "  " << motion_model[7][1]

              << std::endl;
}