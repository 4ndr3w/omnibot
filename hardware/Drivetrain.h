#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include <WPILib/WPILib.h>
#include "../BNO055.h"

class Drivetrain {
	BNO055 *gyro;
	double x,y;
	Victor *left, *right,*front,*back;
	Encoder *frontEncoder,*backEncoder,*rightEncoder,*leftEncoder;
    Drivetrain();
public:
    static Drivetrain* getInstance();

	void drive(double throttle, double turn);
	void drive(double y, double x, double twist);
    void tank(double left, double right);
    
    void reset();
	
	
	double getYaw();

	double getXDistance();
	double getYDistance();
    
    double getLeftDistance();
    double getRightDistance();
	
};

#endif