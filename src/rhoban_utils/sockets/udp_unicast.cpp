#include <iostream>      // std::cerr
#include <arpa/inet.h>   // htons, inet_addr
#include <netinet/in.h>  // sockaddr_in
#include <sys/types.h>   // uint16_t
#include <sys/socket.h>  // socket, sendto
#include <unistd.h>      // close
#include "rhoban_utils/sockets/udp_unicast.h"

namespace rhoban_utils
{
UDPUnicast::UDPUnicast()
{
  sock = socket(AF_INET, SOCK_DGRAM, 0);

  if (sock < 0)
  {
    std::cerr << "Error creating socket for unicast" << std::endl;
    return;
  }
}

void UDPUnicast::send(std::string ip, int port, const unsigned char* data, size_t len)
{
  struct sockaddr_in addr;
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  inet_pton(AF_INET, ip.c_str(), &addr.sin_addr);

  sendto(sock, data, len, 0, (struct sockaddr*)&addr, sizeof(addr));
  close(sock);
}
}  // namespace rhoban_utils