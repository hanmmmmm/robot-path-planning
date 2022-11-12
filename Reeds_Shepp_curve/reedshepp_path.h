#ifndef __REEDSHEPP_PATH__
#define __REEDSHEPP_PATH__
// #ifndef REEDSHEPP_PATH_H
// #define REEDSHEPP_PATH_H

#include <string>
#include <vector>

class ClassReedSheppPath
{
private:

    
public:
    ClassReedSheppPath();
    ~ClassReedSheppPath();

    struct posePerSample
    {
        posePerSample(double xp, double yp, double angle):x(xp),y(yp),theta(angle){};
        double x;
        double y;
        double theta; // radian
    };
    

    struct pathResult
    {
        std::string path_word;
        double path_length_unitless;
        std::vector< posePerSample > path_steps;
        bool valid = false; 
    };
    
    struct PathCollection
    {
        pathResult LpSpLp;
        pathResult LpSpRp;
        pathResult LmSmLm;
        pathResult LmSmRm;
        pathResult RpSpRp;
        pathResult RpSpLp;
        pathResult RmSmRm;
        pathResult RmSmLm;


        pathResult RpLmRp;
        pathResult RmLpRm;
        pathResult RpLmRm;
        pathResult RmLpRp;
        pathResult RpLpRp;
        pathResult RpLpRm;
        pathResult RmLmRp;
        pathResult RmLmRm;

        pathResult LpRpLp;
        pathResult LpRpLm;
        pathResult LpRmLp;
        pathResult LpRmLm;
        pathResult LmRpLp;
        pathResult LmRpLm;
        pathResult LmRmLp;
        pathResult LmRmLm;

        pathResult LpRupLumRm;
        pathResult LmRumLupRp;
        pathResult RpLupRumLm;
        pathResult RmLumRupLp;
        pathResult LpRumLumRp;
        pathResult LmRupLupRm;
        pathResult RpLumRumLp;
        pathResult RmLupRupLm;




        void reset(){
            LpSpLp.path_steps.clear();
            LpSpLp.valid = false;
            LpSpRp.path_steps.clear();
            LpSpRp.valid = false;
            LmSmLm.path_steps.clear();
            LmSmLm.valid = false;
            LmSmRm.path_steps.clear();
            LmSmRm.valid = false;
            RpSpRp.path_steps.clear();
            RpSpRp.valid = false;
            RpSpLp.path_steps.clear();
            RpSpLp.valid = false;
            RmSmRm.path_steps.clear();
            RmSmRm.valid = false;
            RmSmLm.path_steps.clear();
            RmSmLm.valid = false;


            LpRpLp.path_steps.clear();
            LpRpLp.valid = false;
            LpRpLm.path_steps.clear();
            LpRpLm.valid = false;
            LpRmLp.path_steps.clear();
            LpRmLp.valid = false;
            LmRpLm.path_steps.clear();
            LmRpLm.valid = false;
            LpRmLm.path_steps.clear();
            LpRmLm.valid = false;            
            LmRpLp.path_steps.clear();
            LmRpLp.valid = false;            
            LmRmLp.path_steps.clear();
            LmRmLp.valid = false;            
            LmRmLm.path_steps.clear();
            LmRmLm.valid = false;        


            RpLpRp.path_steps.clear();
            RpLpRp.valid = false;
            RpLmRp.path_steps.clear();
            RpLmRp.valid = false;
            RmLpRm.path_steps.clear();
            RmLpRm.valid = false;
            RpLmRm.path_steps.clear();
            RpLmRm.valid = false;
            RmLpRp.path_steps.clear();
            RmLpRp.valid = false;
            RpLpRm.path_steps.clear();
            RpLpRm.valid = false;
            RmLmRp.path_steps.clear();
            RmLmRp.valid = false;
            RmLmRm.path_steps.clear();
            RmLmRm.valid = false;
            

            LpRupLumRm.path_steps.clear();
            LpRupLumRm.valid = false;
            LmRumLupRp.path_steps.clear();
            LmRumLupRp.valid = false;
            RpLupRumLm.path_steps.clear();
            RpLupRumLm.valid = false;
            RmLumRupLp.path_steps.clear();
            RmLumRupLp.valid = false;
            LpRumLumRp.path_steps.clear();
            LpRumLumRp.valid = false;
            LmRupLupRm.path_steps.clear();
            LmRupLupRm.valid = false;
            RpLumRumLp.path_steps.clear();
            RpLumRumLp.valid = false;
            RmLupRupLm.path_steps.clear();
            RmLupRupLm.valid = false;


        }
    };

    void fake();

};

ClassReedSheppPath::ClassReedSheppPath()
{
}

ClassReedSheppPath::~ClassReedSheppPath()
{
}

void ClassReedSheppPath::fake(){

}



#endif