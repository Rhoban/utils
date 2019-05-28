#ifndef RHOBAN_UDPBROADCAST_HPP
#define RHOBAN_UDPBROADCAST_HPP

#include <stdint.h>
#include <vector>

namespace rhoban_utils
{
/**
 * UDPBroadcast
 *
 * Simple UDP non blocking
 * broadcaster
 */
class UDPBroadcast
{
public:
  /**
   * Initializing with listening port
   */
  UDPBroadcast(int portRead, int portWrite);

  /**
   * Close opened connections
   */
  ~UDPBroadcast();

  /**
   * Start or restart the listening and ouput
   * connection
   */
  void openRead();
  void openWrite();

  /**
   * Close listening and output connection
   */
  void closeRead();
  void closeWrite();

  /**
   * Broadcast given UDP message
   */
  void broadcastMessage(const unsigned char* data, size_t len);

  /**
   * Return true of the given bufffer has been
   * filled with incomming data and len has been
   * updated
   */
  bool checkMessage(unsigned char* data, size_t& len, std::string* ip = 0);

private:
  /**
   * Listening and writting port.
   * If _portWrite is -1, packet sending
   * is disabled
   */
  int _portRead;
  int _portWrite;

  /**
   * Incomming and outcomming connection
   */
  int _readFd;
  int _writeFd;

  /**
   * Count the number of send packets since
   * last interface listing
   */
  int _countSend;

  /**
   * Broadcast address list
   */
  std::vector<int> _broadcastAddr;

  /**
   * Retrieve for all interfaces the broadcast address
   */
  void retrieveBroadcastAddress();
};

}  // namespace rhoban_utils

#endif
