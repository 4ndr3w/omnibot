#ifndef NETWORKCOMMUNICATION_H
#define NETWORKCOMMUNICATION_H

#include "OmniBotComm.h"
#include "TCPServer/TCPServer.h"
#include "UDPSender/UDPSender.h"

class NetworkCommunication : public TCPServer {
	int sock;
    UDPSender poseSender;
public:
	NetworkCommunication(int port, int udpPort);
    void handleClient(int sock, sockaddr_in source);
    void handleClientThread(int sock, int addr);

    void updateState(RobotState &state);
};


#endif
