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

        pathResult LpRmL ;
        pathResult LmRpLm;
        pathResult RpLmRp;
        pathResult RmLpRm;
        pathResult LpRmLm;
        pathResult LmRpLp;
        pathResult RpLmRm;
        pathResult RmLpRp;

        // PathCollection(){
        //     LpSpLp.path_word("LpSpLp");
        //     LpSpRp.path_word("LpSpRp");
        //     LmSmLm.path_word("LmSmLm");
        //     LmSmRm.path_word("LmSmRm");
        //     RpSpRp.path_word("RpSpRp");
        //     RpSpLp.path_word("RpSpLp");
        //     RmSmRm.path_word("RmSmRm");
        //     RmSmLm.path_word("RmSmLm");

        //     LpRmL .path_word("LpRmL");
        //     LmRpLm.path_word("LmRpLm");
        //     RpLmRp.path_word("RpLmRp");
        //     RmLpRm.path_word("RmLpRm");
        //     LpRmLm.path_word("LpRmLm");
        //     LmRpLp.path_word("LmRpLp");
        //     RpLmRm.path_word("RpLmRm");
        //     RmLpRp.path_word("RmLpRp");
        // }

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

            LpRmL.path_steps.clear();
            LpRmL.valid = false;
            LmRpLm.path_steps.clear();
            LmRpLm.valid = false;
            RpLmRp.path_steps.clear();
            RpLmRp.valid = false;
            RmLpRm.path_steps.clear();
            RmLpRm.valid = false;
            LpRmLm.path_steps.clear();
            LpRmLm.valid = false;
            LmRpLp.path_steps.clear();
            LmRpLp.valid = false;
            RpLmRm.path_steps.clear();
            RpLmRm.valid = false;
            RmLpRp.path_steps.clear();
            RmLpRp.valid = false;
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