#include "UDPSender.h"

UDPSender::UDPClient::UDPClient(sockaddr_in addr) : client(addr) {
}

sockaddr_in UDPSender::UDPClient::getAddr() {
    return client;
}

bool UDPSender::UDPClient::operator== (const UDPClient& other) {
    return client.sin_addr.s_addr == other.client.sin_addr.s_addr;
}

UDPSender::UDPSender(int port) {
    this->port = port;
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    
    /*sockaddr_in bindaddr;
    int sockaddr_len = sizeof(sockaddr_in);
    bzero((char*)&bindaddr, sockaddr_len);
    bindaddr.sin_family = AF_INET;
    bindaddr.sin_port = htons(port);
    bindaddr.sin_addr.s_addr = INADDR_ANY;
    bindaddr.sin_len = sockaddr_len;

    if ( bind(sock, (sockaddr*)&bindaddr, sockaddr_len) == ERROR ) {
        close(sock);
        exit(1);
    }*/
}


void UDPSender::addClient(int address) {
    sockaddr_in addr;

    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = address;
    addr.sin_len = sizeof(sockaddr_in);

    UDPClient client(addr);

    MutexLock l(lock);
    clients.push_back(client);
    clients.unique();
}

void UDPSender::removeClient(int address) {
    sockaddr_in addr;
    addr.sin_addr.s_addr = address;
    UDPClient client(addr);

    MutexLock l(lock);
    clients.remove(client);
}

void UDPSender::broadcast(caddr_t buf, int len) {
    MutexLock l(lock);
    
    for (std::list<UDPClient>::iterator it=clients.begin(); it != clients.end(); it++) {
        sockaddr_in addr = (*it).getAddr();
        sendto(sock, buf, len, 0, (sockaddr*)&addr, sizeof(sockaddr_in));
    }
}