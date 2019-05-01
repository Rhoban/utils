#include <rhoban_utils/logging/log.h>

#include <rhoban_utils/io_tools.h>
#include <rhoban_utils/util.h>

#include <chrono>
#include <iostream>
#include <regex>

using namespace std::chrono;

namespace rhoban_utils
{

static uint64_t getTS(uint64_t h, uint64_t m, uint64_t s, uint64_t ms, uint64_t offset)
{
  return 1000 * (ms + 1000 * (s + 60 * (m + 60 * h))) + offset;
}

Log::Log()
{
}

int Log::getEntries()
{
  return entries.size();
}

void Log::load(const std::string& filename, uint64_t log_expected_start_ms)
{
  time_point<system_clock> log_start = time_point<system_clock>(milliseconds(log_expected_start_ms));
  uint64_t nb_days = duration_cast<hours>(log_start.time_since_epoch()).count() / 24;
  uint64_t offset = nb_days * 3600 * 24 * 1000;
  
  entries.clear();
  std::string data = file2string(filename);
  std::vector<std::string> lines;
  split(data, '\n', lines);

  std::regex regex("^(.*)\\[(\\d+):(\\d+):(\\d+):(\\d+)\\] (.+)$");
  uint64_t last_ts = 0;
  for (auto& line : lines)
  {
    std::cmatch cm;
    if (std::regex_match(line.c_str(), cm, regex))
    {
      uint64_t hour = std::stoi(cm[2].str());
      uint64_t min = std::stoi(cm[3].str());
      uint64_t sec = std::stoi(cm[4].str());
      uint64_t ms = std::stoi(cm[5].str());

      uint64_t ts = getTS(hour, min, sec, ms, offset);
      //TODO: cover the case where the hours change (ts strongly lower than last_ts)
      entries[ts].push_back(cm[0]);
      last_ts = ts;
    }
    else if (entries.size() > 0)
    {
      // No 
      entries[last_ts].push_back(line);
    }
  }
}

Log::MessageMap Log::entriesBetween(uint64_t start, uint64_t end)
{
  return MessageMap(entries.lower_bound(start), entries.lower_bound(end));
}

}
