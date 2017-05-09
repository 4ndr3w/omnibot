#include "Drivetrain.h"
#include "../PortMap.h"
#include "util.h"

Drivetrain* Drivetrain::getInstance() {
    static Drivetrain drive;
    return &drive;
}

Drivetrain::Drivetrain() {
	gyro = new BNO055();
	
    front = new Victor(FRONT_DRIVE_PWM);
    back = new Victor(BACK_DRIVE_PWM);
    left = new Victor(LEFT_DRIVE_PWM);
    right = new Victor(RIGHT_DRIVE_PWM);
	
    frontEncoder = new Encoder(FRONT_ENCODER_A_DIGITAL, FRONT_ENCODER_B_DIGITAL);
    backEncoder = new Encoder(BACK_ENCODER_A_DIGITAL, BACK_ENCODER_B_DIGITAL);
    leftEncoder = new Encoder(LEFT_ENCODER_A_DIGITAL, LEFT_ENCODER_B_DIGITAL);
    rightEncoder = new Encoder(RIGHT_ENCODER_A_DIGITAL, RIGHT_ENCODER_B_DIGITAL, true);
	
    frontEncoder->SetDistancePerPulse((1/280.0)*PI*4);
    backEncoder->SetDistancePerPulse((1/280.0)*PI*4);
    leftEncoder->SetDistancePerPulse((1/280.0)*PI*4);
    rightEncoder->SetDistancePerPulse((1/280.0)*PI*4);
    
	frontEncoder->Start();
	backEncoder->Start();
	leftEncoder->Start();
	rightEncoder->Start();
}

double Drivetrain::getYaw() {
	return gyro->getYaw();
}

double Drivetrain::getXDistance() {
	return ( frontEncoder->GetDistance() + backEncoder->GetDistance() ) / 2.0;
}

double Drivetrain::getYDistance() { 
	return ( leftEncoder->GetDistance() + rightEncoder->GetDistance() ) / 2.0;
}

double Drivetrain::getLeftDistance() {
    return leftEncoder->GetDistance();
}

double Drivetrain::getRightDistance() {
    return rightEncoder->GetDistance();
}

RobotVelocity Drivetrain::getVelocity() {
    RobotVelocity vel;
    vel.front = frontEncoder->GetRate();
    vel.back = backEncoder->GetRate();
    vel.left = leftEncoder->GetRate();
    vel.right = rightEncoder->GetRate();
    return vel;
}

void Drivetrain::reset() {
    leftEncoder->Reset();
    rightEncoder->Reset();
}

// + = Forward, Right
void Drivetrain::drive(double y, double x, double twist) {
	right->Set(y+twist);
	left->Set(-y+twist);
	
	front->Set(x+twist);
	back->Set(-x+twist);
//	printf("%i %i %i %i\n", leftEncoder->Get(), rightEncoder->Get(), frontEncoder->Get(), backEncoder->Get());
}

void Drivetrain::drive(double throttle, double turn) {
    tank(throttle+turn, throttle-turn);
}

void Drivetrain::tank(double leftVel, double rightVel) {
    double twist = rightVel-leftVel;
    right->Set(-rightVel);
    left->Set(leftVel);
    
    front->Set(-twist);
    back->Set(-twist);
}

void Drivetrain::raw(double left, double right, double front, double back) {
    right->Set(right);
	left->Set(left);
	
	front->Set(front);
	back->Set(back);
}
