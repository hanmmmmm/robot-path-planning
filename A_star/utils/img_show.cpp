#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>


void show_resize_img(cv::Mat* img, float ratio, int waitkeySec, std::string win_name){

    cv::Mat img_new;
    cv::resize(*img, img_new, cv::Size(), ratio, ratio, cv::INTER_NEAREST );

    cv::imshow( win_name, img_new );

    cv::waitKey(waitkeySec);
}

