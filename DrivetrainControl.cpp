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

void DrivetrainControl::setOpenLoop(RobotVelocity goal) {
    mode = OPENLOOP;
    drive->raw(goal.left, goal.right, goal.front, goal.back);
}

void DrivetrainControl::setVelocityGoal(RobotVelocity goal) {
    mode = VELOCITY;
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

void DrivetrainControl::setFrontPIDF(PIDConstants constants) {

}

void DrivetrainControl::setBackPIDF(PIDConstants constants) {

}

void DrivetrainControl::setLeftPIDF(PIDConstants constants) {

}

void DrivetrainControl::setRightPIDF(PIDConstants constants) {

}



ControlMode DrivetrainControl::getMode() {
    return mode;
}

PIDInfo DrivetrainControl::getPIDInfo() {
    PIDInfo data;
    return data;
}