#ifndef BRESENHAM_H
#define BRESENHAM_H

#include <iostream>
#include <vector>


int Sign(int dxy)
{
    if(dxy<0) return -1; 
    else if(dxy>0) return 1; 
    else return 0;
}


std::vector<std::vector<int>> bresenham(int x1, int y1, int x2, int y2)
{
    std::vector<std::vector<int>> out;

    int Dx = x2 - x1;
    int Dy = y2 - y1;

    //# Increments
    int Sx = Sign(Dx); 
    int Sy = Sign(Dy);

    //# Segment length
    Dx = abs(Dx); 
    Dy = abs(Dy); 
    int D = std::max(Dx, Dy);

    //# Initial remainder
    double R = D / 2;

    int X = x1;
    int Y = y1;
    if(Dx > Dy)
    {   
        //# Main loop
        for(int I=0; I<D; I++)
        {   
            std::vector<int> one_px;
            one_px.push_back(X);
            one_px.push_back(Y);
            //# Update (X, Y) and R
            X+= Sx; R+= Dy; //# Lateral move
            if (R >= Dx)
            {
                Y+= Sy; 
                R-= Dx; //# Diagonal move
            }
            out.push_back( one_px );
        }
        
    }
    else
    {   
        //# Main loop
        for(int I=0; I<D; I++)
        {    
            std::vector<int> one_px;
            one_px.push_back(X);
            one_px.push_back(Y);
            //# Update (X, Y) and R
            Y+= Sy; 
            R+= Dx; //# Lateral move
            if(R >= Dy)
            {    
                X+= Sx; 
                R-= Dy; //# Diagonal move
            }
            out.push_back( one_px );
        }
    }
    return out;
}



#endif