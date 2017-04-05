#ifndef UDP_SENDER_H
#define UDP_SENDER_H

#include "sockLib.h"
#include "inetLib.h"
#include "strLib.h"
#include "Mutex.h"
#include <list>

class UDPSender {
    class UDPClient {
        sockaddr_in client;
        public:
            UDPClient(sockaddr_in addr);
            sockaddr_in getAddr();

            bool operator== (const UDPClient& other);
    };

    std::list<UDPClient> clients;
    int port;
    int sock;
    Mutex lock;
    public:
        UDPSender(int port);
        void addClient(int address);
        void removeClient(int address);
        void broadcast(caddr_t buf, int len);
};

#endif
