#include "define.h"

namespace autol_driver
{
  UDPSocket::UDPSocket()
  {
  }

  UDPSocket::~UDPSocket() {}
  int UDPSocket::CreateSocket()
  {
    udp_socket_ = socket(PF_INET, SOCK_DGRAM, 0);
    if (udp_socket_ != 0)
      return 0;
    else
      return -1;
  }

  int UDPSocket::Bind(unsigned short port)
  {
    struct sockaddr_in add;
    int32_t bindRetVal = 0;
    memset(&add, 0, sizeof(add));
    add.sin_family = AF_INET;
    add.sin_addr.s_addr = htonl(INADDR_ANY);
    add.sin_port = htons(port);
    bindRetVal = bind(udp_socket_, (struct sockaddr *)&add, sizeof(sockaddr));
    return bindRetVal;
  }

  int UDPSocket::SetSocketBuffer(int size)
  {
    int rc, rbuffer, wbuffer;
    rbuffer = wbuffer = size;
    rc = setsockopt(udp_socket_, SOL_SOCKET, SO_RCVBUF, (char *)&rbuffer,
                    sizeof(rbuffer));
    return 1;
  }

  int UDPSocket::PrintSocketBuffer() { return 0; }

  int UDPSocket::SetTimeout(timeval tv)
  {
    struct timeval optVal = tv;
    int optLen = sizeof(optVal);

    unsigned long dw = (tv.tv_sec * 1000) + ((tv.tv_usec + 999) / 1000);
    if (setsockopt(udp_socket_, SOL_SOCKET, SO_RCVTIMEO, (const char *)&dw,
                   optLen) < 0)
    {
      return -1;
    }
    return 0;
  }

  int UDPSocket::RecvFrom(char *buffer, int len,
                          int flags) // OOB = 0, Non-Blocking = 2
  {
    sockaddr_in from;
    socklen_t size = sizeof(from);

    ssize_t ret =
        recvfrom(udp_socket_, buffer, len, flags, (sockaddr *)&from, &size);

    if (ret < 0)
    {
      return -1;
    }
    return ret;
  }

  int UDPSocket::RecvFrom(autol_msgs::msg::AutolPacket *packet, int len,
                          int flags) // OOB = 0, Non-Blocking = 2
  {
    sockaddr_in from;
    socklen_t size = sizeof(from);

    ssize_t ret = recvfrom(udp_socket_, &packet->data[0], len, flags,
                           (sockaddr *)&from, &size);

    if (ret < 0)
    {
      return -1;
    }
    return ret;
  }

  int UDPSocket::CloseSocket()
  {
    if (udp_socket_ != 0)
      return close(udp_socket_);
    return 0;
  }

} // namespace autol_driver
