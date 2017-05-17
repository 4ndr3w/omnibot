#include "PoseCalculator.h"
#include <WPILib/WPILib.h>
#include "constants.h"
#include <math.h>

PoseCalculator* PoseCalculator::getInstance() {
    static PoseCalculator pose;
    return &pose;
}


PoseCalculator::PoseCalculator() : x(0), y(0) {
    lock = semMCreate(SEM_Q_FIFO);
    drive = Drivetrain::getInstance();
}


RobotPose PoseCalculator::getPose() {
    RobotPose pose;
    
    semTake(lock, WAIT_FOREVER);
    pose.x = x;
    pose.y = y;
    pose.theta = theta;
    
    pose.forwardDistance = drive->getYDistance();
    
    pose.vx = vx;
    pose.vy = vy;
    pose.vth = vth;
    semGive(lock);
    
    return pose;
}

RobotVelocity PoseCalculator::getVelocity() {
    return drive->getVelocity();
}

void PoseCalculator::reset() {
    semTake(lock, WAIT_FOREVER);
    x = y = theta = 0;
    vx = vy = vth;
    semGive(lock);
}

void PoseCalculator::update() {
    static double lastTime = Timer::GetFPGATimestamp();
    static double lastXPos = 0;//drive->getXDistance();
    static double lastYPos = drive->getYDistance();
    static double lastTheta = (360-drive->getYaw());
    
	double xPos = 0;//drive->getXDistance();
	double yPos = drive->getYDistance();

    double theta = (360-drive->getYaw());
	double thetaRad = theta * (PI/180); 
    
    double dt = Timer::GetFPGATimestamp()-lastTime;
    
    double dxRot = 0;
    double dyRot = 0;

    double dTheta = (theta - lastTheta);
    double dx = xPos - lastXPos;
    double dy = yPos - lastYPos;
    
    // Compute distance traveled, rotated from robot frame to world frame
    dxRot = dx * cos(thetaRad) - dy * sin(thetaRad);
    dyRot = dx * sin(thetaRad) + dy * cos(thetaRad);

    lastTime = Timer::GetFPGATimestamp();
    lastXPos = xPos;
    lastYPos = yPos;
    lastTheta = theta;


    semTake(lock, WAIT_FOREVER);
    this->theta = theta;
    x += dxRot;
    y += dyRot;
    
    if ( dt != 0 ) {
		vx = dxRot/dt;
		vy = dyRot/dt;
		vth = dTheta/dt;
    }
    semGive(lock);
}
