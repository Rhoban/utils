#include "rhoban_utils/timing/time_stamp.h"

#include <ctime>

using namespace std::chrono;

namespace rhoban_utils
{
TimeStamp::TimeStamp()
{
}
TimeStamp::TimeStamp(const time_point<steady_clock>& timePoint) : monotonic_clock(timePoint)
{
  uint64_t us_base = duration_cast<duration<uint64_t, std::micro>>(monotonic_clock.time_since_epoch()).count();
  uint64_t us_epoch = us_base + getSteadyClockOffset();
  utc_clock = time_point<system_clock>(microseconds(us_epoch));
}

TimeStamp::TimeStamp(const time_point<steady_clock>& monotonic, const time_point<system_clock>& system)
  : monotonic_clock(monotonic), utc_clock(system)
{
}

TimeStamp TimeStamp::now()
{
  return TimeStamp(steady_clock::now(), system_clock::now());
}

TimeStamp TimeStamp::addMS(int64_t ms_offset) const
{
  duration<int64_t, std::milli> offset(ms_offset);
  return TimeStamp(monotonic_clock + offset, utc_clock + offset);
}

TimeStamp TimeStamp::fromMS(unsigned long msSinceEpoch)
{
  return TimeStamp(time_point<steady_clock>(milliseconds(msSinceEpoch)));
}

double TimeStamp::getTimeSec(bool utc) const
{
  return getTimeMS(utc) / 1000;
}

double TimeStamp::getTimeMS(bool utc) const
{
  if (utc)
  {
    return duration_cast<duration<double, std::milli>>(utc_clock.time_since_epoch()).count();
  }
  return duration_cast<duration<double, std::milli>>(monotonic_clock.time_since_epoch()).count();
}

uint64_t TimeStamp::getTimeUS(bool utc) const
{
  if (utc)
  {
    return duration_cast<duration<uint64_t, std::micro>>(utc_clock.time_since_epoch()).count();
  }
  return duration_cast<duration<uint64_t, std::micro>>(monotonic_clock.time_since_epoch()).count();
}

std::string getFormattedTime()
{
  system_clock::time_point now = system_clock::now();
  time_t tt = system_clock::to_time_t(now);
  struct tm tm;
  localtime_r(&tt, &tm);
  char buffer[80];  // Buffer is big enough
  sprintf(buffer, "%04d_%02d_%02d_%02dh%02dm%02ds", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min,
          tm.tm_sec);
  return std::string(buffer);
}

int64_t getSteadyClockOffset()
{
  int64_t steady_ts = duration_cast<duration<int64_t, std::micro>>(steady_clock::now().time_since_epoch()).count();
  int64_t system_ts = duration_cast<duration<int64_t, std::micro>>(system_clock::now().time_since_epoch()).count();
  return system_ts - steady_ts;
}

}  // namespace rhoban_utils

double diffSec(const rhoban_utils::TimeStamp& src, const rhoban_utils::TimeStamp& dst, bool utc)
{
  return dst.getTimeSec(utc) - src.getTimeSec(utc);
}

double diffMs(const rhoban_utils::TimeStamp& src, const rhoban_utils::TimeStamp& dst, bool utc)
{
  return dst.getTimeMS(utc) - src.getTimeMS(utc);
}

bool operator<(const rhoban_utils::TimeStamp& ts1, const rhoban_utils::TimeStamp& ts2)
{
  return diffMs(ts1, ts2) > 0;
}
bool operator>(const rhoban_utils::TimeStamp& ts1, const rhoban_utils::TimeStamp& ts2)
{
  return diffMs(ts1, ts2) < 0;
}
