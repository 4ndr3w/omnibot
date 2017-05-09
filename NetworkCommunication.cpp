#include "NetworkCommunication.h"
#include "sockLib.h"
#include "inetLib.h"
#include "strLib.h"
#include "taskLib.h"
#include "OmniBotComm.h"
#include <stdio.h>
#include "DrivetrainControl.h"
#include "PoseCalculator.h"

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
    int bytes = 0;
    while ( (bytes = read(sock, (char*)&cmd, sizeof(RobotMessage))) != -1 )
    {
        if ( bytes != sizeof(RobotMessage) )
            continue;
        RobotResponse status = {true};

		if ( cmd.type == MSG_SET_PID_FRONT )
            driveControl->setFrontPIDF(cmd.data.pid);
        else if ( cmd.type == MSG_SET_PID_BACK )
            driveControl->setBackPIDF(cmd.data.pid);
        else if ( cmd.type == MSG_SET_PID_LEFT )
            driveControl->setLeftPIDF(cmd.data.pid);
        else if ( cmd.type == MSG_SET_PID_RIGHT )
            driveControl->setRightPIDF(cmd.data.pid);
        else if ( cmd.type == MSG_RESET_POSE )
            PoseCalculator::getInstance()->reset();
        else if ( cmd.type == MSG_VELOCITY_GOAL )
            driveControl->setVelocityGoal(cmd.data.goal);
        else if ( cmd.type == MSG_OPENLOOP_GOAL )
            driveControl->setOpenLoop(cmd.data.goal);
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
