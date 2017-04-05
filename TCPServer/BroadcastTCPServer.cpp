#include "TCPServer.h"
#include "sockLib.h"
#include "strLib.h"
#include "stdio.h"

BroadcastTCPServer::BroadcastTCPServer(int port) : TCPServer(port),
  firstClient(NULL), lastClient(NULL)
{
  mutex = semMCreate(SEM_Q_FIFO);
}

void BroadcastTCPServer::handleClient(int sock, sockaddr_in source) {
  ClientNode *client = new ClientNode;
  bzero((char*)client, sizeof(ClientNode));

  client->socket = sock;

  semTake(mutex, WAIT_FOREVER);
  if ( firstClient == NULL )
    firstClient = lastClient = client;
  else {
    lastClient->next = client;
    client->prev = lastClient;

    lastClient = client;
  }
  semGive(mutex);
}

void BroadcastTCPServer::broadcast(char* message, int len)
{
  semTake(mutex, WAIT_FOREVER);
  ClientNode *cur = firstClient;
  while (cur != NULL)
  {
    if ( send(cur->socket, message, len, 0) == ERROR )
    {
      printf("send failed, removing\n");
      if ( cur->next != NULL )
        cur->next->prev = cur->prev;
      else
        lastClient = cur->prev;

      if ( cur->prev != NULL )
        cur->prev->next = cur->next;
      else
        firstClient = cur->next;

      ClientNode *old = cur;
      cur = cur->next;
      delete old;
    }
    else
      cur = cur->next;
  }
  semGive(mutex);
}
