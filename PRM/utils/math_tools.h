#ifndef MATH_TOOLS_H
#define MATH_TOOLS_H

#include <array>
#include <math.h>

std::array<int, 3> fine_to_grid(const std::array<float, 3> c , float dist_ratio, float ang_ratio )
{
    std::array<int, 3> ct;
    ct[0] = int(c[0] / dist_ratio);
    ct[1] = int(c[1] / dist_ratio);
    ct[2] = int(c[2] / ang_ratio);
    return ct;
}

float rectify_angle_rad(float ang)
{
    while (ang > 2 * M_PI)
        ang -= 2 * M_PI;
    while (ang < 0)
        ang += 2 * M_PI;
    return ang;
}


int compute_h_cost_Euclidean(const std::array<int, 2> n, const std::array<int, 2> g)
{
    float h = 0;

    int dx = n[0] - g[0];
    int dy = n[1] - g[1];

    h = sqrt(dx * dx + dy * dy) * 1;

    return int(h);
}

int compute_h_cost_Manhattan(const std::array<int, 2> n, const std::array<int, 2> g)
{
    int h = 0;

    int dx = abs(n[0] - g[0]);
    int dy = abs(n[1] - g[1]);

    h = (dx + dy) * 1;

    return int(h);
}

int compute_h_cost_Chebyshev(const std::array<int, 2> n, const std::array<int, 2> g)
{
    int h = 0;

    int dx = abs(n[0] - g[0]);
    int dy = abs(n[1] - g[1]);

    h = std::max(dx, dy) * 1;

    return int(h);
}




#endif