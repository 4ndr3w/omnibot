#include <WPILib/WPILib.h>
#include "NetworkCommunication.h"
#include "PoseCalculator.h"
#include "DrivetrainControl.h"

class RobotMain : public SimpleRobot {

	Notifier controlLoops;
	NetworkCommunication netComm;
	
	public:

		static void UpdateLoops(void* param) {
			PoseCalculator::getInstance()->update();
			DrivetrainControl::getInstance()->update();
		}

		RobotMain() :
			controlLoops(UpdateLoops, NULL),
			netComm(1337, 1338)
		{
			// 100hz control loop
			// BNO055 only updates at 100hz, so not much point in going faster 
			controlLoops.StartPeriodic(1.0/100.0); 
		}

		void Disabled() {
			controlLoops.Stop();
			while(!IsEnabled())
				Wait(0.01);
		}

		/*void TeleopPeriodic() {
			double y = dbc(js.GetRawAxis(2), 0.05);
			double x = dbc(js.GetRawAxis(1), 0.05);
			double heading = (360-drive.getYaw()) * (PI/180);

			double yRot = x * sin(heading) + y * cos(heading);
			double xRot = x * cos(heading) - y * sin(heading);

			drive.drive(yRot, xRot, dbc(js.GetRawAxis(3), 0.05));
		}*/


		void Autonomous() {  
			RobotState state;  
			PoseCalculator *pose = PoseCalculator::getInstance();
			DrivetrainControl *drive = DrivetrainControl::getInstance();
			while (IsEnabled()) {
				state.control = drive->getPIDInfo();
				state.pose = pose->getPose();
				netComm.updateState(state);
                Wait(1.0/100.0);
			}
		}
};


START_ROBOT_CLASS(RobotMain);
