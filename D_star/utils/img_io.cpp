#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <unistd.h>
#include <iostream>

void load_img(std::string img_path, cv::Mat* img , bool* succ){

    char cwd[256];
    getcwd(cwd, 256);

    std::string path = cwd;
    while( path.back() != '/'){
        path.pop_back();
    }
    path += "image/";
    path += img_path;

    std::cout << path << std::endl;

    *img = cv::imread(path, cv::IMREAD_COLOR);

    if(img->empty()){
        *succ = false;
    }
    else{
        *succ = true;
    }

}


cv::Mat setup_map( std::string img_path){
    bool succ = false;
    cv::Mat img;

    load_img(img_path, &img, &succ);

    if(!succ){
        throw std::runtime_error("Can not open image");
    }
    return img;
}




void load_resize_img(std::string img_path, float ratio , cv::Mat* img , bool* succ){
    *img = cv::imread(img_path, cv::IMREAD_COLOR);

    if(img->empty()){
        *succ = false;
    }
    else{
        *succ = true;

        cv::Mat img_rsz;

        cv::resize(*img, img_rsz, cv::Size(), ratio, ratio, cv::INTER_NEAREST);

        *img = img_rsz;
    }

}


void show_resize_img(cv::Mat* img, float ratio, int waitkeySec, std::string win_name){

    cv::Mat img_new;
    cv::resize(*img, img_new, cv::Size(), ratio, ratio, cv::INTER_NEAREST );

    cv::imshow( win_name, img_new );

    cv::waitKey(waitkeySec);
}

