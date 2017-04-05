#include <WPILib/WPILib.h>
#include "hardware/Drivetrain.h"
#include "OmniBotComm.h"

#ifndef POSE_CALC_H
#define POSE_CALC_H


class PoseCalculator {
private:
    SEM_ID lock;
    double x,y,theta;
    double vx,vy,vth;
    Drivetrain *drive;
    
    PoseCalculator();
     
public:
    static PoseCalculator* getInstance();

    RobotPose getPose();
    
    void update();
    
    
};

#endif