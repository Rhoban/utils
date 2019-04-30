#pragma once

#include <chrono>
#include <string>

namespace rhoban_utils
{
class TimeStamp : public std::chrono::time_point<std::chrono::steady_clock>
{
public:
  TimeStamp();
  TimeStamp(const std::chrono::time_point<std::chrono::steady_clock>& timePoint);
  static TimeStamp now();

  static TimeStamp fromMS(unsigned long msSinceEpoch);

  double getTimeSec() const;
  double getTimeMS() const;
};

/**
 * Uses system_clock to extract a formatted time: format is:
 * - YYYY_MM_DD_HHhMMmSSs Ex: 2016_03_25_35h23m12s
 * Function is reentrant
 */
std::string getFormattedTime();

/**
 * Return the offset from steady_clock to system_clock in us:
 * steady_clock + offset = system_clock
 */
int64_t getSteadyClockOffset();

}  // namespace rhoban_utils

double diffSec(const rhoban_utils::TimeStamp& src, const rhoban_utils::TimeStamp& dst);
double diffMs(const rhoban_utils::TimeStamp& src, const rhoban_utils::TimeStamp& dst);

bool operator<(const rhoban_utils::TimeStamp& ts1, const rhoban_utils::TimeStamp& ts2);
bool operator>(const rhoban_utils::TimeStamp& ts1, const rhoban_utils::TimeStamp& ts2);
