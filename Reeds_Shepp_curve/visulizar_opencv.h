#ifndef VISULIZAR_OPENCV_H
#define VISULIZAR_OPENCV_H 


#include <iostream>
#include <array>
// #include <queue>
// #include <unordered_map>
// #include <map>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
// #include <algorithm>
// #include <set>

#include "reedsshepp.h"
#include "reedshepp_path.h"

class ClassRScurveOpencv
{
private:

    const std::string window_name = "map";

    double map_left_meter_ = -5.0; // 
    double map_down_meter_ = -5.0;
    int map_width_pixel_  = 600;
    int map_height_pixel_ = 600;

    double meter_to_pixel_ratio_ = 50.0 ; // meter x this -> pixel 

    cv::Mat map_;  

    cv::Mat map_backup_;   

    int legend_pose_u_ = 0;
    int legend_pose_v_ = 30;
    int legend_height_ = 30;

    std::unordered_map<std::string, cv::Scalar> all_path_colors;
    

    void meter_to_pixel( double x, double y, int& u, int& v ); // xy are meters; uv are pixel.
    

    void add_legend( std::string name, double length );

public:
    ClassRScurveOpencv();
    ~ClassRScurveOpencv();

    void init();

    void show(bool manual );

    void draw_vector_on_map(std::array<double, 3> vect_meter );

    void draw_path_by_cvline( ClassReedSheppPath::pathResult path_holder  );
    void draw_path_by_cvPoint( ClassReedSheppPath::pathResult path_holder  );
};

ClassRScurveOpencv::ClassRScurveOpencv()
{
    map_backup_ = cv::Mat::zeros(cv::Size(map_width_pixel_ , map_height_pixel_) , CV_8UC3);
    map_backup_.setTo( cv::Scalar(225,225,225) );

    init();

    all_path_colors["LpSpLp"] = cv::Scalar(0,255,0);
    all_path_colors["LpSpRp"] = cv::Scalar(255,0,0);
    all_path_colors["LmSmLm"] = cv::Scalar(0,0,255);
    all_path_colors["LmSmRm"] = cv::Scalar(0,200,100);
    all_path_colors["RpSpRp"] = cv::Scalar(80,0,155);
    all_path_colors["RpSpLp"] = cv::Scalar(255,0,0);
    all_path_colors["RmSmRm"] = cv::Scalar(0,0,255);
    all_path_colors["RmSmLm"] = cv::Scalar(0,200,100);

    all_path_colors["LpRmL"] = cv::Scalar(0,255,0);
    all_path_colors["LmRpLm"] = cv::Scalar(255,0,0);
    all_path_colors["RpLmRp"] = cv::Scalar(0,0,255);
    all_path_colors["RmLpRm"] = cv::Scalar(0,200,100);
    all_path_colors["LpRmLm"] = cv::Scalar(80,0,155);
    all_path_colors["LmRpLp"] = cv::Scalar(255,0,0);
    all_path_colors["RpLmRm"] = cv::Scalar(0,0,255);
    all_path_colors["RmLpRp"] = cv::Scalar(0,200,100);

    all_path_colors["LpRupLumRm"] = cv::Scalar(0,255,0);
    all_path_colors["LmRumLupRp"] = cv::Scalar(255,0,0);
    all_path_colors["RpLupRumLm"] = cv::Scalar(0,0,255);
    all_path_colors["RmLumRupLp"] = cv::Scalar(0,200,100);
    all_path_colors["LpRumLumRp"] = cv::Scalar(80,0,155);
    all_path_colors["LmRupLupRm"] = cv::Scalar(255,0,0);
    all_path_colors["RpLumRumLp"] = cv::Scalar(0,0,255);
    all_path_colors["RmLupRupLm"] = cv::Scalar(0,200,100);

    all_path_colors["LpRm90SmRm"] = cv::Scalar(0,255,0);
    all_path_colors["LmRp90SpRp"] = cv::Scalar(255,0,0);
    all_path_colors["RpLm90SmLm"] = cv::Scalar(0,0,255);
    all_path_colors["RmLp90SpLp"] = cv::Scalar(0,200,100);

    all_path_colors["LpRm90SmLm"] = cv::Scalar(80,0,155);
    all_path_colors["LmRp90SpLp"] = cv::Scalar(255,0,0);
    all_path_colors["RpLm90SmRm"] = cv::Scalar(0,0,255);
    all_path_colors["RmLp90SpRp"] = cv::Scalar(0,200,100);

    all_path_colors["RpSpLp90Rm"] = cv::Scalar(206, 160, 247);
    all_path_colors["RmSmLm90Rp"] = cv::Scalar(114, 176, 25);
    all_path_colors["LpSpRp90Lm"] = cv::Scalar(207, 30, 52);
    all_path_colors["LmSmRm90Lp"] = cv::Scalar(84, 152, 251);

    all_path_colors["LpSpLp90Rm"] = cv::Scalar(147, 157, 48);
    all_path_colors["LmSmLm90Rp"] = cv::Scalar(195, 26, 87);
    all_path_colors["RpSpRp90Lm"] = cv::Scalar(25, 247, 39);
    all_path_colors["RmSmRm90Lp"] = cv::Scalar(97, 249, 88);


}

ClassRScurveOpencv::~ClassRScurveOpencv()
{
}

void ClassRScurveOpencv::init(){
    map_ = map_backup_.clone();
    legend_pose_v_ = 30; 
}

void ClassRScurveOpencv::show(bool manual ){
    cv::imshow( window_name , map_ );
    int delay ;
    if(manual) delay = 0;
    else  delay = 30;
    cv::waitKey(delay);
}


void ClassRScurveOpencv::meter_to_pixel( double x, double y, int& u, int& v )
{
    double dx = x - map_left_meter_ ;
    double dy = y - map_down_meter_ ; 
    int du = dx * meter_to_pixel_ratio_ ;
    int dv = dy * meter_to_pixel_ratio_ ;
    u = du;
    v = map_height_pixel_ - dv; 
}


void ClassRScurveOpencv::draw_vector_on_map( std::array<double, 3> vect_meter )
{
    double x_meter = vect_meter[0];
    double y_meter = vect_meter[1];
    double theta = vect_meter[2];

    double vect_length_meter  = 1;

    double x_end_meter = x_meter + vect_length_meter * cos(theta);
    double y_end_meter = y_meter + vect_length_meter * sin(theta);

    int u_end, v_end, u_start, v_start;
    meter_to_pixel( x_end_meter, y_end_meter, u_end,   v_end );
    meter_to_pixel( x_meter,     y_meter,     u_start, v_start );

    // std::cout << "\ndraw_vector_on_map(): " << std::endl;
    // std::cout << "From " << x_meter << " "<< y_meter << " "<< theta << " == ";
    // std::cout << u_start << " "<< v_start << std::endl;
    // std::cout << "To   " << x_end_meter << " "<< y_end_meter << " "<< theta << " == ";
    // std::cout << u_end << " "<< v_end << std::endl;

    cv::line(map_, cv::Point(u_start, v_start), cv::Point(u_end, v_end), cv::Vec3b(0,0,0), 3);

}

void ClassRScurveOpencv::draw_path_by_cvline( ClassReedSheppPath::pathResult path_holder ){
    // cv::Vec3b cvcolor = { uchar( color[0]), uchar( color[1]), uchar( color[2]) };

    if( ! path_holder.valid ){ return; }
    int pt_a_x, pt_a_y, pt_b_x, pt_b_y; 
    bool first_point = true;
    for ( auto step : path_holder.path_steps){
        meter_to_pixel( step.x, step.y, pt_b_x, pt_b_y);
        if( first_point ){
            first_point = false;
        }
        else{
            cv::line(map_, cv::Point(pt_a_x, pt_a_y), cv::Point(pt_b_x, pt_b_y), all_path_colors[path_holder.path_word], 1);
        }
        pt_a_x = pt_b_x;
        pt_a_y = pt_b_y;
    }
    add_legend( path_holder.path_word, path_holder.path_length_unitless );
}


void ClassRScurveOpencv::draw_path_by_cvPoint( ClassReedSheppPath::pathResult path_holder ){
    if( ! path_holder.valid ){ return; }
    int pt_x, pt_y; 
    for ( auto step : path_holder.path_steps){
        meter_to_pixel( step.x, step.y, pt_x, pt_y);
        cv::circle(map_, cv::Point(pt_x, pt_y), 2, all_path_colors[path_holder.path_word] , -1 ); // fill the circle to be point. 
    }

    add_legend( path_holder.path_word, path_holder.path_length_unitless );
}


void ClassRScurveOpencv::add_legend( std::string name, double length ){
        
        std::ostringstream str;
        str << name << " " << std::to_string( length).substr(0,5);
        cv::putText(map_ , str.str(), cv::Point(legend_pose_u_, legend_pose_v_),cv::FONT_HERSHEY_DUPLEX,1, all_path_colors[name] ,2,false);

        legend_pose_v_ += legend_height_ ;
}

#endif 


