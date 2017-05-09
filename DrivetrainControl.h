#include "hardware/Drivetrain.h"
#include "SimplePID.h"
#include "semLib.h"
#include "OmniBotComm.h"
#include <queue>
#include "VXAtomic.h"

#ifndef DRIVE_CONTROL_H
#define DRIVE_CONTROL_H

enum ControlMode {
    OPENLOOP,
    VELOCITY
};


class DrivetrainControl {
    SEM_ID lock;

    VXAtomic<ControlMode> mode;
    
    SimplePID left;
    SimplePID right;
    SimplePID front;
    SimplePID back;

    Drivetrain *drive;

    DrivetrainControl();

public:
    static DrivetrainControl* getInstance();
    
    void setVelocityGoal(RobotVelocity goal);
    void setOpenLoop(RobotVelocity goal);

    void update();

    void setFrontPIDF(PIDConstants constants);
    void setBackPIDF(PIDConstants constants);
    void setLeftPIDF(PIDConstants constants);
    void setRightPIDF(PIDConstants constants);

    ControlMode getMode();
    PIDInfo getPIDInfo();
};

#endif
