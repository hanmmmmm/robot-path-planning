#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <stdexcept>

#include "utils/img_io.h"
#include "d_star.h"




void find_path(int startnode[], int goalnode[], cv::Mat map)
{
    D_star path_finder(startnode, goalnode, map);

    path_finder.search();
    path_finder.get_path();

    int i = 0;

    while(path_finder.path.size() > 0){

        path_finder.move_robot();

        // add obstacles
        if(i==3)
        {
            std::vector<int> obs = {13,12,15,22};
            path_finder.update_obs_map( obs );
        }

        // add obstacles
        if(i==10)
        {
            std::vector<int> obs = {20,19,22,30};
            path_finder.update_obs_map( obs );

            obs = {42,31,46,33};
            path_finder.update_obs_map( obs );

        }

        // add obstacles
        if(i==22)
        {
            std::vector<int> obs = {28,24,30,33};
            path_finder.update_obs_map( obs );
        }

        i++;

    }


}



int main(){

    std::string img_path = "map3.png";

    cv::Mat img = setup_map(img_path);
    

    int startnode[2] = {3, 8};  // start position, {x ,y}
    int goalnode[2] = {44, 48}; // goal  position, {x ,y}



    goalnode[0] = 47;  // start position, x
    goalnode[1] = 35;  // start position, x
    find_path( startnode , goalnode, img);

}




