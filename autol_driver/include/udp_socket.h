#ifndef AUTOL_ROS_UDP_SOCKET_H_
#define AUTOL_ROS_UDP_SOCKET_H_
#include "define.h"

namespace autol_driver {
  class UDPSocket{ 
  public:
    UDPSocket();
    ~UDPSocket();

    int CreateSocket();
    int CloseSocket();
    int RecvFrom(char *buffer, int len,
                int flags = 0); // OOB = 0, Non-Blocking = 2
    int RecvFrom(autol_msgs::msg::AutolPacket *buffer, int len, int flags = 0);
    int Bind(unsigned short port);
    int SetTimeout(timeval tv);
    int SetSocketBuffer(int size);
    int PrintSocketBuffer();
    int udp_socket_;
  };
} // namespace autol_driver
#endif
