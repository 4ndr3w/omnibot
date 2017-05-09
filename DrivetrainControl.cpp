#include "DrivetrainControl.h"
#include "constants.h"
#include "PoseCalculator.h"
#include <math.h>

DrivetrainControl* DrivetrainControl::getInstance() {
    static DrivetrainControl control;
    return &control;
}

DrivetrainControl::DrivetrainControl() :
    lock(semMCreate(SEM_Q_FIFO)), 
    mode(OPENLOOP),
    left(), right(), front(), back(),
    drive(Drivetrain::getInstance()) {
}

void DrivetrainControl::setVelocityGoal(RobotVelocity goal) {
    mode = VELOCITY;
    drive->raw(goal.left, goal.right, goal.front, goal.back);
}

void DrivetrainControl::setOpenLoop(RobotVelocity goal) {
    mode = OPENLOOP;
    left.setSetpoint(goal.left);
    right.setSetpoint(goal.right);
    front.setSetpoint(goal.front);
    back.setSetpoint(goal.back);
}

void DrivetrainControl::update() {
    semTake(lock, WAIT_FOREVER);
    if ( mode == VELOCITY ) {
        RobotVelocity actual = PoseCalculator::getInstance()->getVelocity();
        
        drive->raw(
            left.calculate(actual.left),
            right.calculate(actual.right),
            front.calculate(actual.front),
            back.calculate(actual.back)
        );
    }
    semGive(lock);
}

void DrivetrainControl::setFrontPIDF(double kP, double kI, double kD, double kF) {

}

void DrivetrainControl::setBackPIDF(double kP, double kI, double kD, double kF) {

}

void DrivetrainControl::setLeftPIDF(double kP, double kI, double kD, double kF) {

}

void DrivetrainControl::setRightPIDF(double kP, double kI, double kD, double kF) {

}



ControlMode DrivetrainControl::getMode() {
    return mode;
}

PIDInfo DrivetrainControl::getPIDInfo() {
    PIDInfo data = {linearActual, linearSetpoint, angularActual, angularSetpoint};
    return data;
}