#ifndef AUTOL_ROS_UDP_SOCKET_HPP_
#define AUTOL_ROS_UDP_SOCKET_HPP_
#include "define.hpp"
class UDPSocket
{
public:
    UDPSocket(){}
    ~UDPSocket(){}

    int CreateSocket();
    int Bind(unsigned short port);
    int SetSocketBuffer(int size);
    int SetTimeout(timeval tv);
    int CloseSocket();
    int RecvFrom(char *buffer, int len, sockaddr_in *from, int flags = 0); // OOB = 0, Non-Blocking = 2
    int32_t socket_;
};

int UDPSocket::CreateSocket()
{
    socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    if (socket_ == INVALID_SOCKET)
    {
        std::cerr << "Error creating server socket" << std::endl;
        return -1;
    }
    return 0;
}
int UDPSocket::Bind(unsigned short port)
{
    int32_t retVal = 0;
    // Bind the socket to a port
    sockaddr_in serverAddr;
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    serverAddr.sin_port = htons(port);
    retVal = bind(socket_,(struct sockaddr *)(&serverAddr), sizeof(serverAddr));
    if (retVal == -1)
    {
        std::cerr << "Error binding server socket" << std::endl;
        close(socket_);
        return -1;
    }
    return 0;
}

int UDPSocket::SetSocketBuffer(int size)
{
    int rc, rbuffer, wbuffer;
    rbuffer = wbuffer = size;
    rc = setsockopt(socket_, SOL_SOCKET, SO_RCVBUF, (char *)&rbuffer, sizeof(rbuffer));
    UNUSED(rc);
    return 1;
}

int UDPSocket::SetTimeout(timeval tv)
{
    struct timeval optVal = tv;

    int optLen = sizeof(optVal);

    unsigned long dw = (tv.tv_sec * 1000) + ((tv.tv_usec + 999) / 1000);
    if (setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, (const char *)&dw, optLen) < 0)
    {
        return -1;
        perror("Error");
    }

    return 0;
}
int UDPSocket::RecvFrom(char *buffer, int len, sockaddr_in *from, int flags) // OOB = 0, Non-Blocking = 2
{
    socklen_t size = sizeof(*from);
    int ret = recvfrom(socket_, buffer, len, flags,  (sockaddr *)from, &size);
    return ret;
}
int UDPSocket::CloseSocket()
{
    // Close the sockets
    if (socket_ <= 0)
    {
        close(socket_);
    }

    return 0;
}
#endif