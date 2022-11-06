// #include "../reedsshepp.h"
// #include <opencv2/imgproc.hpp>

// void ReedsSheppClass::draw_start_goal_on_map(int arrow_length )
// {
//     cv::Vec3b color1 = {0,0,0}; //{30, 80, 255};
//     cv::Vec3b color2 = {0,0,0}; //{170, 30, 225};

//     float s_theta = start_pose[2];
//     float g_theta = goal_pose[2];

//     // int arrow_length = 25;

//     int s_x_new = start_pose[0] + arrow_length * cos(s_theta);
//     int s_y_new = start_pose[1] + arrow_length * sin(s_theta);

//     int g_x_new = goal_pose[0] + arrow_length * cos(g_theta);
//     int g_y_new = goal_pose[1] + arrow_length * sin(g_theta);

//     cv::line(map_for_view, cv::Point(s_x_new, s_y_new), cv::Point(start_pose[0], start_pose[1]),
//                 color1, 4);

//     cv::line(map_for_view, cv::Point(g_x_new, g_y_new), cv::Point(goal_pose[0], goal_pose[1]),
//                 color2, 4);

//     cv::circle(map_for_view, cv::Point(start_pose[0], start_pose[1]), 5, color1, -1);
//     cv::circle(map_for_view, cv::Point(goal_pose[0], goal_pose[1]  ), 5, color2, -1);
// }


// void ReedsSheppClass::draw_vector_on_map( std::array<float, 3> vect )
// {
//     cv::Vec3b color1 = {0, 255, 0};

//     float s_theta = vect[2];

//     int arrow_length  = 25;

//     int s_x_new = vect[0] + arrow_length * cos(s_theta);
//     int s_y_new = vect[1] + arrow_length * sin(s_theta);

//     cv::line(map_for_view, cv::Point(s_x_new, s_y_new), cv::Point(vect[0], vect[1]),
//                 color1, 4);

//     cv::circle(map_for_view, cv::Point(vect[0], vect[1]), 5, color1, -1);

// }



// // void ReedsSheppClass::draw_arc(std::array<float, 3> node_pose, std::array<float, 3> next_pose, int radius, float arc_angle, int motion_type, int thickness, cv::Scalar color)
// // {
// //     float xn = node_pose[0];
// //     float yn = node_pose[1];
// //     float an_rad = node_pose[2];
// //     float an_deg = an_rad * 180.0/M_PI;
// //     arc_angle *= 180.0/M_PI;
// //     cv::Size  sz(radius, radius);

// //     float arc_start_angle ,arc_end_angle;
// //     int dx = float(radius) * cos(an_rad + M_PI / 2);
// //     int dy = float(radius) * sin(an_rad + M_PI / 2);
// //     int xc, yc;

// //     switch (motion_type)
// //     {
// //     case 1:
// //     case 4:
// //         cv::line(map_for_view, cv::Point(int(xn), int(yn)), cv::Point(next_pose[0], next_pose[1]), color, thickness );
// //         break;
// //     case 0:
// //         xc = xn - dx;
// //         yc = yn - dy;
// //         arc_start_angle = an_deg + 90;
// //         arc_end_angle = arc_start_angle -  arc_angle ;
// //         cv::ellipse(map_for_view, cv::Point(xc, yc), sz, 0.0, arc_start_angle, arc_end_angle , color, thickness );
// //         break;
// //     case 5:
// //         xc = xn - dx;
// //         yc = yn - dy;
// //         arc_start_angle = an_deg + 90 + arc_angle;
// //         arc_end_angle = arc_start_angle -  arc_angle ;
// //         cv::ellipse(map_for_view, cv::Point(xc, yc), sz, 0.0, arc_start_angle, arc_end_angle , color, thickness );
// //         break;
// //     case 2:
// //         xc = xn + dx;
// //         yc = yn + dy;
// //         arc_start_angle = an_deg - 90;
// //         arc_end_angle = arc_start_angle +  arc_angle ;
// //         cv::ellipse(map_for_view, cv::Point(xc, yc), sz, 0.0, arc_start_angle, arc_end_angle , color, thickness );
// //         break;
// //     case 3:
// //         xc = xn + dx;
// //         yc = yn + dy;
// //         arc_start_angle = an_deg - 90 - arc_angle;
// //         arc_end_angle = arc_start_angle +  arc_angle ;
// //         cv::ellipse(map_for_view, cv::Point(xc, yc), sz, 0.0, arc_start_angle, arc_end_angle , color, thickness );
// //         break;
// //     }

// //     // std::cout << "angle start: " << arc_start_angle << " -- " << arc_end_angle << std::endl;
// //     // cv::circle(map_for_view, cv::Point(xc, yc), 1, cv::Scalar(0,0,255),-1);
// //     // cv::ellipse(map_for_view, cv::Point(xc, yc), sz, 0.0, arc_start_angle, arc_end_angle , color, thickness );

// // }



