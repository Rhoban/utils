#pragma once

#include <chrono>
#include <string>

namespace rhoban_utils
{
class TimeStamp
{
public:
  TimeStamp();
  /**
   * Guesses system clock based on current offset
   */
  TimeStamp(const std::chrono::time_point<std::chrono::steady_clock>& timePoint);
  TimeStamp(const std::chrono::time_point<std::chrono::steady_clock>& monotonic,
            const std::chrono::time_point<std::chrono::system_clock>& system);
  static TimeStamp now();

  TimeStamp addMS(int64_t ms_offset) const;

  /**
   * Guesses system clock based on current offset
   */
  static TimeStamp fromMS(unsigned long msSinceEpoch);

  double getTimeSec(bool utc = false) const;
  double getTimeMS(bool utc = false) const;
  /**
   * Return time in micro-seconds since epoch
   */
  uint64_t getTimeUS(bool utc = false) const;

private:
  std::chrono::time_point<std::chrono::steady_clock> monotonic_clock;
  std::chrono::time_point<std::chrono::system_clock> utc_clock;
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

double diffSec(const rhoban_utils::TimeStamp& src, const rhoban_utils::TimeStamp& dst, bool utc = false);
double diffMs(const rhoban_utils::TimeStamp& src, const rhoban_utils::TimeStamp& dst, bool utc = false);

bool operator<(const rhoban_utils::TimeStamp& ts1, const rhoban_utils::TimeStamp& ts2);
bool operator>(const rhoban_utils::TimeStamp& ts1, const rhoban_utils::TimeStamp& ts2);
