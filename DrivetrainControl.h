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
    OMNI_CLOSEDLOOP,
    TANK_CLOSEDLOOP,
    TANK_PROFILE
};


class DrivetrainControl {
    Drivetrain *drive;
    
    SEM_ID lock;
    SEM_ID profileQueueMutex;

    VXAtomic<ControlMode> mode;
    
    SimplePID linearPID;
    SimplePID angularPID;

    std::queue<PathPoint> profileBuffer;
    double linearProfile_kV, angularProfile_kV;
    DrivetrainControl();

    VXAtomic<double> linearSetpoint;
    VXAtomic<double> linearActual;

    VXAtomic<double> angularSetpoint;
    VXAtomic<double> angularActual;
public:
    static DrivetrainControl* getInstance();
    
    void setOmniGoal(bool closedLoop, double x, double y, double theta);
    void setDifferentialGoal(bool closedLoop, double linear, double angular);
    void startProfiledDifferentialGoal();
    
    void pushProfilePoint(PathPoint p);
    void clearProfileBuffer();

    void setDifferentialLinearPID(double p, double i, double d, double f);
    void setDifferentialAngularPID(double p, double i, double d, double f);

    ControlMode getMode();
    double getLinearSetpoint();
    double getLinearActual();
    double getAngularSetpoint();
    double getAngularActual();

    PIDInfo getPIDInfo();

    void update();
};

#endif
