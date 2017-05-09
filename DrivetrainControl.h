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
    
    void DrivetrainControl::setVelocityGoal(RobotVelocity goal);
    void DrivetrainControl::setOpenLoop(RobotVelocity goal);

    void DrivetrainControl::update();

    void DrivetrainControl::setFrontPIDF(double kP, double kI, double kD, double kF);
    void DrivetrainControl::setBackPIDF(double kP, double kI, double kD, double kF);
    void DrivetrainControl::setLeftPIDF(double kP, double kI, double kD, double kF);
    void DrivetrainControl::setRightPIDF(double kP, double kI, double kD, double kF);
    ControlMode DrivetrainControl::getMode();
    PIDInfo DrivetrainControl::getPIDInfo();

    void update();
};

#endif
