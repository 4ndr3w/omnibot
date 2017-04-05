#ifndef OMNIBOT_COMM
#define OMNIBOT_COMM

#include <stdint.h>

struct RobotPose {
    double x;
    double y;
    double theta;
    
    double vx;
    double vy;
    double vth;
    
    double forwardDistance;
};

struct PIDInfo {
    double linearActual;
    double linearSetpoint;
    
    double angularActual;
    double angularSetpoint;
};

struct RobotState {
    uint8_t mode;
    
    RobotPose pose;
    PIDInfo control;
};

struct ProfilePoint {
	double pos;
	double vel;
	double acc;
};

#define MSG_DIFFERENTIAL_GOAL 1
struct DriveDifferentialGoal {
    bool useClosedLoop;
    
    double distanceGoal;
    double angleGoal;
};

#define MSG_OMNI_GOAL 2
struct DriveOmniGoal {
    double x;
    double y;
    double theta;
    
    bool useClosedLoop;
};

#define MSG_PROFILED_DIFFERENTIAL_GOAL 3

#define MSG_APPEND_PROFILE_BUFFER 4
struct PathPoint {
	ProfilePoint linear;
    ProfilePoint angular;
	
	bool reinit;
    bool isLast;
	bool velOnly;
};

#define MSG_CLEAR_PROFILE_BUFFER 5

#define MSG_SET_DIFFERENTIAL_LINEAR_PID 6
#define MSG_SET_DIFFERENTIAL_ANGULAR_PID 7
struct PIDConstants {
    double p;
    double i;
    double d;
    double f;
};

#define MSG_RESET_ENCODERS 8
#define MSG_RESET_GYRO 9
#define MSG_RESET_POSE 10

struct RobotMessage {
    uint8_t type;
    union {
        PathPoint point;
        DriveOmniGoal omniGoal;
        DriveDifferentialGoal differentialGoal;   
        PIDConstants pid;
    } data;
};

struct RobotResponse {
    bool result;
};



#endif
