#ifndef RHOBAN_UDPUNICAST_HPP
#define RHOBAN_UDPUNICAST_HPP

#include <string>
#include <stdint.h>
#include <vector>

namespace rhoban_utils
{
/**
 * UDPBroadcast
 *
 * Simple UDP unicaster
 */
class UDPUnicast
{
public:
  UDPUnicast();
  void send(std::string ip, int port, const unsigned char* data, size_t len);

protected:
  int sock;
};
}  // namespace rhoban_utils

#endif  // RHOBAN_UDPUNICAST_HPP