#ifndef ANGLE_THETA_H
#define ANGLE_THETA_H

#include <math.h>

class Angle_Theta
{
private:
    /* data */
public:
    Angle_Theta(float theta);
    ~Angle_Theta();
    float value;
    float degree();
};

Angle_Theta::Angle_Theta(float theta=0.0)
{
    while (theta < 0)
    {
        theta += 2.0 * M_PI;  
    }
    while (theta > 2.0 * M_PI)
    {
        theta -= 2.0 * M_PI;  
    }

    value = theta;
    
}

Angle_Theta::~Angle_Theta()
{
}


float Angle_Theta::degree(){
    return (value*180.0)/M_PI;
}


#endif



