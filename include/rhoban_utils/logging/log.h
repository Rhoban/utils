#pragma once

#include <map>
#include <string>
#include <vector>

namespace rhoban_utils
{

/**
 * Represents a log written by rhoban_utils::Logger as a mapping between entries and time
 */
class Log
{
public:
  typedef std::map<uint64_t, std::vector<std::string>> MessageMap;
  
  Log();
  /**
   * If log_expected_start is provided, then uses it to determine the day the log has started
   * otherwise, simply consider that the day is Jan 1st 1970
   * log_expected_start is in ms
   */
  void load(const std::string& log, uint64_t log_expected_start = 0);

  MessageMap entriesBetween(uint64_t start, uint64_t end);

  int getEntries();

protected:
  /**
   * Entries ordered by time_stamp in ms
   */
  MessageMap entries;
};

}
