#ifndef IMG_IO_H 
#define IMG_IO_H

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>


void load_img(std::string img_path, cv::Mat* img , bool* succ);

cv::Mat setup_map( std::string img_path);

void load_resize_img(std::string img_path, float ratio , cv::Mat* img , bool* succ);

void show_resize_img(cv::Mat* img, float ratio, int waitkeySec, std::string win_name);


#endif 