#include <WPILib/WPILib.h>
#include "NetworkCommunication.h"
#include "PoseCalculator.h"
#include "DrivetrainControl.h"

class RobotMain : public RobotBase {

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
		}

		void StartCompetition() {

			double period = 1.0/100.0;
			while ( true ) {
				if ( IsDisabled() ) {
					controlLoops.Stop();
					m_ds->InDisabled(true);
					while ( IsDisabled() ) {
						Run();
						Wait(period);
					}
					m_ds->InDisabled(false);
				}

				if ( IsAutonomous() ) {
					controlLoops.StartPeriodic(1.0/100.0); 

					m_ds->InAutonomous(true);
					while ( IsAutonomous() ) {
						Run();
						Wait(period);
					}
					m_ds->InAutonomous(false);
				}
			}
		}

		void Run() {
			static RobotState state;  
			static PoseCalculator *pose = PoseCalculator::getInstance();
			static DrivetrainControl *drive = DrivetrainControl::getInstance();
			
			state.control = drive->getPIDInfo();
			state.pose = pose->getPose();
			netComm.updateState(state);
		}
};


START_ROBOT_CLASS(RobotMain);
