#include "NetworkCommunication.h"
#include "sockLib.h"
#include "inetLib.h"
#include "strLib.h"
#include "taskLib.h"
#include "OmniBotComm.h"
#include <stdio.h>
#include "DrivetrainControl.h"

NetworkCommunication::NetworkCommunication(int port, int udpPort) : TCPServer(port), poseSender(udpPort) {
}

void spawnNetworkCommunicationHandleClient(void* ptr, int sock, int addr) {
    ((NetworkCommunication*)ptr)->handleClientThread(sock, addr);
}


void NetworkCommunication::handleClient(int sock, sockaddr_in source) {
    taskSpawn((char*)"NetworkCommunicationClient", 101, VX_FP_TASK, 2048, (FUNCPTR)spawnNetworkCommunicationHandleClient, (int)this,sock, source.sin_addr.s_addr,0,0,0,0,0,0,0);
}


void NetworkCommunication::handleClientThread(int sock, int addr) {
    RobotMessage cmd;
    DrivetrainControl *driveControl = DrivetrainControl::getInstance();

    poseSender.addClient(addr);
    while ( read(sock, (char*)&cmd, sizeof(RobotMessage)) != -1 )
    {
        RobotResponse status = {true};

		if ( cmd.type == MSG_DIFFERENTIAL_GOAL ) {
            driveControl->setDifferentialGoal(cmd.data.differentialGoal.useClosedLoop, cmd.data.differentialGoal.distanceGoal, cmd.data.differentialGoal.angleGoal);
        }
        else if ( cmd.type == MSG_OMNI_GOAL ) {
            driveControl->setOmniGoal(cmd.data.omniGoal.useClosedLoop, cmd.data.omniGoal.x, cmd.data.omniGoal.y, cmd.data.omniGoal.theta);
        }
        else if ( cmd.type == MSG_PROFILED_DIFFERENTIAL_GOAL ) {
            driveControl->startProfiledDifferentialGoal();
        }
        else if ( cmd.type == MSG_APPEND_PROFILE_BUFFER ) {
            driveControl->pushProfilePoint(cmd.data.point);
        }
        else if ( cmd.type == MSG_CLEAR_PROFILE_BUFFER ) {
            driveControl->clearProfileBuffer();
        }
        else if ( cmd.type == MSG_SET_DIFFERENTIAL_LINEAR_PID ) {
            driveControl->setDifferentialLinearPID(cmd.data.pid.p, cmd.data.pid.i, cmd.data.pid.d, cmd.data.pid.f);
        }
        else if ( cmd.type == MSG_SET_DIFFERENTIAL_ANGULAR_PID ) {
            driveControl->setDifferentialAngularPID(cmd.data.pid.p, cmd.data.pid.i, cmd.data.pid.d, cmd.data.pid.f);
        }
        else
            status.result = false;

        send(sock, (char*)&status, sizeof(RobotResponse), 0);
    }
    close(sock);
    poseSender.removeClient(addr);
}

void NetworkCommunication::updateState(RobotState &state) {
    poseSender.broadcast((caddr_t)&state, sizeof(RobotState));
}
