#include "DrivetrainControl.h"
#include "constants.h"
#include "PoseCalculator.h"
#include <math.h>

DrivetrainControl* DrivetrainControl::getInstance() {
    static DrivetrainControl control;
    return &control;
}

DrivetrainControl::DrivetrainControl() {
    mode = OPENLOOP;
    drive = Drivetrain::getInstance();
    lock = semMCreate(SEM_Q_FIFO);
    profileQueueMutex = semMCreate(SEM_Q_FIFO);
}

void DrivetrainControl::setOmniGoal(bool closedLoop, double x, double y, double theta) {
    semTake(lock, WAIT_FOREVER);
    if ( closedLoop ) {
        // Not yet implemented
        mode = OPENLOOP;
        drive->drive(0, 0);
    }
    else {
        double heading = (360-drive->getYaw()) * (PI/180);

        double yRot = x * sin(heading) + y * cos(heading);
        double xRot = x * cos(heading) - y * sin(heading);
        drive->drive(yRot, xRot, theta);
    }
    semGive(lock);
}

void DrivetrainControl::setDifferentialGoal(bool closedLoop, double linear, double angular) {
    semTake(lock, WAIT_FOREVER);
    if ( closedLoop ) {
         mode = TANK_CLOSEDLOOP;
         linearPID.setSetpoint(linear);
         angularPID.setSetpoint(angular);
    }
    else {
        mode = OPENLOOP;
        drive->drive(linear+angular, linear-angular);
    }
    semGive(lock);
}

void DrivetrainControl::startProfiledDifferentialGoal() {
    semTake(lock, WAIT_FOREVER);
    mode = TANK_PROFILE;
    semGive(lock);
}

void DrivetrainControl::update() {
    semTake(lock, WAIT_FOREVER);
    RobotPose pose = PoseCalculator::getInstance()->getPose();
    if ( mode == TANK_CLOSEDLOOP ) {
        double linearOut = linearPID.calculate(pose.forwardDistance);
        
        if ( linearOut > 1 )
            linearOut = 1;
        else if ( linearOut < -1 )
            linearOut = -1;

        double angularOut = angularPID.calculate(pose.theta);

        drive->drive(linearOut+angularOut, linearOut-angularOut);
    }
    else if ( mode == TANK_PROFILE ) {
        // Time for a new point
        semTake(profileQueueMutex, WAIT_FOREVER);
        PathPoint point = profileBuffer.front();
        profileBuffer.pop();
        semGive(profileQueueMutex);

        linearPID.setSetpoint(point.linear.pos);
        angularPID.setSetpoint(point.angular.pos);

        double linearOut = point.linear.vel * linearProfile_kV + linearPID.calculate(pose.forwardDistance);

        if ( linearOut > 1 )
            linearOut = 1;
        else if ( linearOut < -1 ) 
            linearOut = -1;

        double angularOut = point.angular.pos * angularProfile_kV + angularPID.calculate(pose.theta);

        drive->drive(linearOut+angularOut, linearOut-angularOut);
    }
    semGive(lock);
}

void DrivetrainControl::clearProfileBuffer() {
    semTake(profileQueueMutex, WAIT_FOREVER);
    while ( !profileBuffer.empty() )
        profileBuffer.pop();
    semGive(profileQueueMutex);
}

void DrivetrainControl::pushProfilePoint(PathPoint p) {
    semTake(profileQueueMutex, WAIT_FOREVER);
    profileBuffer.push(p);
    semGive(profileQueueMutex);
}

void DrivetrainControl::setDifferentialLinearPID(double p, double i, double d, double f) {
    semTake(lock, WAIT_FOREVER);
    linearPID.setPIDF(p,i,d,0);
    linearProfile_kV = f;
    semGive(lock);
}

void DrivetrainControl::setDifferentialAngularPID(double p, double i, double d, double f) {
    semTake(lock, WAIT_FOREVER);
    angularPID.setPIDF(p,i,d,0);
    angularProfile_kV = f;
    semGive(lock);
}

ControlMode DrivetrainControl::getMode() {
    return mode;
}

double DrivetrainControl::getLinearActual() {
    return linearActual;
}

double DrivetrainControl::getLinearSetpoint() {
    return linearSetpoint;
}

double DrivetrainControl::getAngularActual() {
    return angularActual;
}

double DrivetrainControl::getAngularSetpoint() {
    return angularSetpoint;
}

PIDInfo DrivetrainControl::getPIDInfo() {
    PIDInfo data = {linearActual, linearSetpoint, angularActual, angularSetpoint};
    return data;
}