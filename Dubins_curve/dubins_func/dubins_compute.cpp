#include "../dubins.h"
// #include "utils/math_tools.h"
#include <opencv2/imgproc.hpp>
#include <math.h>
// #include "../utils/math_tools.h"


int Dubins::check_collision(float x, float y, int scan_range)
{
    int xstart = std::max(int(x) - scan_range, 0);
    int ystart = std::max(int(y) - scan_range, 0);
    int xrange = std::min(scan_range * 2, fine_map_width - xstart - 1);
    int yrange = std::min(scan_range * 2, fine_map_height - ystart - 1);
    cv::Rect roi(xstart, ystart, xrange, yrange);

    cv::Mat local_map = map_collision(roi);

    cv::Scalar local_map_sum = cv::sum(local_map);

    return local_map_sum[0];
}

void Dubins::map_inflation(cv::Mat *mapIn, cv::Mat *mapOut, int infla_size)
{
    const int erode_size = infla_size;
    cv::Mat erode_element = cv::getStructuringElement(0, cv::Size(2 * erode_size + 1, 2 * erode_size + 1), cv::Point(erode_size, erode_size));
    cv::erode(*mapIn, *mapOut, erode_element);
}





void Dubins::compute_LSL_path(float alpha, float beta, float d, std::vector<float>& path_angles_seq)
{
    float atan_part = std::atan( (cos(beta) - cos(alpha)) / (d+ sin(alpha) - sin(beta)) );

    float t = -alpha + rectify_angle_rad( atan_part);
    float q = beta - rectify_angle_rad( atan_part);

    float p = sqrt( 2+d*d-2*cos(alpha-beta) + 2*d*( sin(alpha) - sin(beta)) );

    path_angles_seq.push_back(t);
    path_angles_seq.push_back(p);
    path_angles_seq.push_back(q);
}


void Dubins::compute_RSR_path(float alpha, float beta, float d, std::vector<float>& path_angles_seq)
{
    float atan_part = std::atan( (cos(alpha) - cos(beta)) / (d - sin(alpha) + sin(beta)) );

    float t = alpha - rectify_angle_rad( atan_part);
    float q = -1 * rectify_angle_rad(beta) + rectify_angle_rad( atan_part);

    float p = sqrt( 2+d*d-2*cos(alpha-beta) + 2*d*( sin(beta) - sin(alpha)) );

    path_angles_seq.push_back(t);
    path_angles_seq.push_back(p);
    path_angles_seq.push_back(q);
}


void Dubins::compute_LSR_path(float alpha, float beta, float d, std::vector<float>& path_angles_seq)
{

}


void Dubins::compute_RSL_path(float alpha, float beta, float d, std::vector<float>& path_angles_seq)
{

}


void Dubins::compute_LRL_path(float alpha, float beta, float d, std::vector<float>& path_angles_seq)
{

}


void Dubins::compute_RLR_path(float alpha, float beta, float d, std::vector<float>& path_angles_seq)
{

}







float Dubins::rectify_angle_rad(float ang)
{
    while (ang > 2 * M_PI)
        ang -= 2 * M_PI;
    while (ang < 0)
        ang += 2 * M_PI;
    return ang;
}


float Dubins::compute_h_cost_Euclidean(const std::array<float, 3> n, const std::array<float, 3> g)
{
    float h = 0;

    float dx = n[0] - g[0];
    float dy = n[1] - g[1];

    h = sqrt(dx * dx + dy * dy) * 1;

    return (h);
}
