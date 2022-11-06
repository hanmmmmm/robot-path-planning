#include "../reedsshepp.h"
// #include "utils/math_tools.h"
#include <opencv2/imgproc.hpp>
#include <math.h>
// #include "../utils/math_tools.h"
#include <numeric>




// void ReedsSheppClass::compute_LSL_path(float alpha, float beta, float d, std::vector<float>& path_angles_seq, float& total_length)
// {
//     float atan_part = std::atan( (cos(beta) - cos(alpha)) / (d+ sin(alpha) - sin(beta)) );

//     float t = -alpha + rectify_angle_rad( atan_part);
//     float q = beta - rectify_angle_rad( atan_part);

//     float p = sqrt( 2+d*d-2*cos(alpha-beta) + 2*d*( sin(alpha) - sin(beta)) );

//     // total_length = rectify_angle_rad(t)+p+rectify_angle_rad(q);

//     path_angles_seq.push_back(t);
//     path_angles_seq.push_back(p);
//     path_angles_seq.push_back(q);
//     path_angles_seq[0] = rectify_angle_rad(path_angles_seq[0]);
//     path_angles_seq[1] = path_angles_seq[1]*turning_raius;
//     path_angles_seq[2] = rectify_angle_rad(path_angles_seq[2]);


//     if(2*M_PI-path_angles_seq[0] < 0.1)
//     {
//         path_angles_seq[0] = 0.0;
//     }
//     if(2*M_PI-path_angles_seq[2] < 0.1)
//     {
//         path_angles_seq[2] = 0.0;
//     }

//     total_length = path_angles_seq[0] + p + path_angles_seq[2];

//     std::cout << "LSL  t " << path_angles_seq[0]  << "  p " << p  << "  q " << path_angles_seq[2]  << std::endl;

    
// }


// void ReedsSheppClass::compute_RSR_path(float alpha, float beta, float d, std::vector<float>& path_angles_seq, float& total_length)
// {
//     float atan_part = std::atan( (cos(alpha) - cos(beta)) / (d - sin(alpha) + sin(beta)) );

//     float t = alpha - rectify_angle_rad( atan_part);
//     float q = -1 * rectify_angle_rad(beta) + rectify_angle_rad( atan_part);

//     float p = sqrt( 2+d*d-2*cos(alpha-beta) + 2*d*( sin(beta) - sin(alpha)) );

    


//     // total_length = rectify_angle_rad(t)+p+rectify_angle_rad(q);

//     path_angles_seq.push_back(t);
//     path_angles_seq.push_back(p);
//     path_angles_seq.push_back(q);
//     path_angles_seq[0] = rectify_angle_rad(path_angles_seq[0]);
//     path_angles_seq[2] = rectify_angle_rad(path_angles_seq[2]);
//     path_angles_seq[1] = path_angles_seq[1]*turning_raius;

//     if(2*M_PI-path_angles_seq[0] < 0.1)
//     {
//         path_angles_seq[0] = 0.0;
//     }
//     if(2*M_PI-path_angles_seq[2] < 0.1)
//     {
//         path_angles_seq[2] = 0.0;
//     }

//     total_length = path_angles_seq[0] + p + path_angles_seq[2];

// std::cout << "RSR  t " << path_angles_seq[0]  << "  p " << p  << "  q " << path_angles_seq[2]  << std::endl;
// }


// void ReedsSheppClass::compute_LSR_path(float alpha, float beta, float d, std::vector<float>& path_angles_seq, float& total_length)
// {
//     float atan_part = std::atan( (-cos(alpha) - cos(beta)) / (d + sin(alpha) + sin(beta)) );

//     float p = sqrt( -2+d*d + 2*cos(alpha-beta) + 2*d*( sin(beta) + sin(alpha)) );

//     float t = rectify_angle_rad( - alpha  + atan_part - std::atan( -2/p ) );
//     float q = - rectify_angle_rad(beta) + atan_part - rectify_angle_rad( std::atan( -2/p ) );

    
//     total_length = rectify_angle_rad(t)+p+rectify_angle_rad(q);

    
//     path_angles_seq.push_back(t);
//     path_angles_seq.push_back(p);
//     path_angles_seq.push_back(q);
//     path_angles_seq[0] = rectify_angle_rad(path_angles_seq[0]);
//     path_angles_seq[2] = rectify_angle_rad(path_angles_seq[2]);
//     path_angles_seq[1] = path_angles_seq[1]*turning_raius;

    
//     if(2*M_PI-path_angles_seq[0] < 0.1)
//     {
//         path_angles_seq[0] = 0.0;
//     }
//     if(2*M_PI-path_angles_seq[2] < 0.1)
//     {
//         path_angles_seq[2] = 0.0;
//     }

//     total_length = path_angles_seq[0] + p + path_angles_seq[2];


//     std::cout << "LSR  t " << path_angles_seq[0]  << "  p " << p  << "  q " << path_angles_seq[2]  << std::endl;
// }


// void ReedsSheppClass::compute_RSL_path(float alpha, float beta, float d, std::vector<float>& path_angles_seq, float& total_length)
// {
//     float atan_part = std::atan( ( cos(alpha) + cos(beta)) / (d - sin(alpha) - sin(beta)) );

//     float p = sqrt( -2+d*d + 2*cos(alpha-beta) - 2*d*( sin(beta) + sin(alpha)) );

//     float t =  alpha - atan_part + rectify_angle_rad(std::atan( 2/p ) );
//     float q = rectify_angle_rad(beta) - atan_part + rectify_angle_rad( std::atan( 2/p ));
//     // float q = rectify_angle_rad(-beta) + atan_part - rectify_angle_rad( std::atan( -2/p ));

    

//     total_length = rectify_angle_rad(t)+p+rectify_angle_rad(q);

//     path_angles_seq.push_back(t);
//     path_angles_seq.push_back(p);
//     path_angles_seq.push_back(q);
//     path_angles_seq[0] = rectify_angle_rad(path_angles_seq[0]);
//     path_angles_seq[2] = rectify_angle_rad(path_angles_seq[2]);
//     path_angles_seq[1] = path_angles_seq[1]*turning_raius;

//     if(2*M_PI-path_angles_seq[0] < 0.1)
//     {
//         path_angles_seq[0] = 0.0;
//     }
//     if(2*M_PI-path_angles_seq[2] < 0.1)
//     {
//         path_angles_seq[2] = 0.0;
//     }

//     total_length = path_angles_seq[0] + p + path_angles_seq[2];

//     std::cout << "RSL  t " << path_angles_seq[0]  << "  p " << p  << "  q " << path_angles_seq[2]  << std::endl;
// }


// void ReedsSheppClass::compute_LRL_path(float alpha, float beta, float d, std::vector<float>& path_angles_seq, float& total_length)
// {
//     // float p = rectify_angle_rad( acos( ( 6-d*d +2*cos(alpha-beta) + 2*d*(sin(alpha) - sin(beta)) )/8 ) ) ;

//     // float p = rectify_angle_rad(acos(  ( 6-d*d +2*cos(alpha-beta) + 2*d*(sin(alpha) - sin(beta)) )/8 ) ) ;
//     // float t = rectify_angle_rad( -alpha + std::atan( (-cos(alpha) + cos(beta)) / (d+sin(alpha)-sin(beta))) + p/2.0 ) ;
//     // float q = rectify_angle_rad(beta) - alpha +  rectify_angle_rad(2*p);

//     float p = rectify_angle_rad( 2*M_PI - acos(  ( 6-d*d +2*cos(alpha-beta) + 2*d*(-sin(alpha) + sin(beta)) )/8.0 ) ) ;
//     float t = rectify_angle_rad( -alpha - std::atan( ( cos(alpha) - cos(beta)) / (d+sin(alpha)-sin(beta))) + p/2.0 ) ;
//     float q = rectify_angle_rad(beta) - alpha -t +  rectify_angle_rad(p);


//     total_length = rectify_angle_rad(t)+p+rectify_angle_rad(q);

//     path_angles_seq.push_back(t);
//     path_angles_seq.push_back(p);
//     path_angles_seq.push_back(q);
//     path_angles_seq[0] = rectify_angle_rad(path_angles_seq[0]);
//     path_angles_seq[2] = rectify_angle_rad(path_angles_seq[2]);
//     path_angles_seq[1] = rectify_angle_rad(path_angles_seq[1]);

//     if(2*M_PI-path_angles_seq[0] < 0.1)
//     {
//         path_angles_seq[0] = 0.0;
//     }
//     if(2*M_PI-path_angles_seq[2] < 0.1)
//     {
//         path_angles_seq[2] = 0.0;
//     }

//     total_length = path_angles_seq[0] + p + path_angles_seq[2];

//     std::cout << "LRL  t " << path_angles_seq[0]  << "  p " << p  << "  q " << path_angles_seq[2]  << std::endl;

// }


// void ReedsSheppClass::compute_RLR_path(float alpha, float beta, float d, std::vector<float>& path_angles_seq, float& total_length)
// {
//     float p = rectify_angle_rad( 2*M_PI - acos(  ( 6-d*d +2*cos(alpha-beta) + 2*d*(sin(alpha) - sin(beta)) )/8.0 ) ) ;
//     float t = rectify_angle_rad( alpha - std::atan( ( cos(alpha) - cos(beta)) / (d-sin(alpha)+sin(beta))) + rectify_angle_rad(p/2.0) ) ;
//     float q = rectify_angle_rad( -beta + alpha -t +  rectify_angle_rad(p));

    


    

//     path_angles_seq.push_back(t);
//     path_angles_seq.push_back(p);
//     path_angles_seq.push_back(q);
//     path_angles_seq[0] = rectify_angle_rad(path_angles_seq[0]);
//     path_angles_seq[2] = rectify_angle_rad(path_angles_seq[2]);
//     path_angles_seq[1] = rectify_angle_rad(path_angles_seq[1]);

//     if(2*M_PI-path_angles_seq[0] < 0.1)
//     {
//         path_angles_seq[0] = 0.0;
//     }
//     if(2*M_PI-path_angles_seq[2] < 0.1)
//     {
//         path_angles_seq[2] = 0.0;
//     }

//     total_length = path_angles_seq[0] + p + path_angles_seq[2];

//     std::cout << "RLR  t " << path_angles_seq[0]  << "  p " << p  << "  q " << path_angles_seq[2]  << std::endl;

// }







// float ReedsSheppClass::rectify_angle_rad(float ang)
// {
//     while (ang > 2 * M_PI)
//         ang -= 2 * M_PI;
//     while (ang < 0)
//         ang += 2 * M_PI;
//     return ang;
// }


// float ReedsSheppClass::compute_h_cost_Euclidean(const std::array<float, 3> n, const std::array<float, 3> g)
// {
//     float h = 0;

//     float dx = n[0] - g[0];
//     float dy = n[1] - g[1];

//     h = sqrt(dx * dx + dy * dy) * 1;

//     return (h);
// }
