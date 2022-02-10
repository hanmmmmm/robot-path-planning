#include "../rrt.h"
// #include "utils/math_tools.h"
#include <opencv2/imgproc.hpp>

// int RRT::check_collision(float x, float y, int scan_range)
// {
//     int xstart = std::max(int(x) - scan_range, 0);
//     int ystart = std::max(int(y) - scan_range, 0);
//     int xrange = std::min(scan_range * 2, fine_map_width - xstart - 1);
//     int yrange = std::min(scan_range * 2, fine_map_height - ystart - 1);
//     cv::Rect roi(xstart, ystart, xrange, yrange);

//     cv::Mat local_map = map_collision(roi);

//     cv::Scalar local_map_sum = cv::sum(local_map);

//     return local_map_sum[0];
// }

// void RRT::map_inflation(cv::Mat *mapIn, cv::Mat *mapOut, int infla_size)
// {
//     const int erode_size = infla_size;
//     cv::Mat erode_element = cv::getStructuringElement(0, cv::Size(2 * erode_size + 1, 2 * erode_size + 1), cv::Point(erode_size, erode_size));
//     cv::erode(*mapIn, *mapOut, erode_element);
// }

// int RRT::improve_hcost(std::array<float, 3> node_pose, int hcost, int scan_range, bool same_steer)
// {
//     float nb_x = node_pose[0];
//     float nb_y = node_pose[1];

//     if ( check_collision(nb_x, nb_y, scan_range) > 0)
//         hcost *= 1.2;

//     if (!same_steer)
//         hcost *= 1.05;

//     return hcost;
// }
