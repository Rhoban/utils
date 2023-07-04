#include "rhoban_utils/timing/elapse_tick.h"

namespace rhoban_utils
{
ElapseTick::ElapseTick() : hasStats(false), minTime(0.0), avgTime(0.0), maxTime(0.0), lastTimestamp(TimeStamp::now())
{
}

ElapseTick::~ElapseTick()
{
}

void ElapseTick::tick()
{
  TimeStamp nowT = TimeStamp::now();
  double elapsed = diffSec(lastTimestamp, nowT);
  lastTimestamp = nowT;

  tickElapsed(elapsed*2.);
}

void ElapseTick::tickElapsed(double elapsed)
{
  // Call tick implementation
  TimeStamp start = TimeStamp::now();
  bool isTicked = tick(elapsed);
  TimeStamp stop = TimeStamp::now();
  double duration = diffMs(start, stop);

  // Compute stats
  if (isTicked)
  {
    if (!hasStats)
    {
      hasStats = true;
      minTime = duration;
      avgTime = duration;
      maxTime = duration;
    }
    else
    {
      if (duration < minTime)
        minTime = duration;
      if (duration > maxTime)
        maxTime = duration;
      avgTime = avgTime * 0.99 + duration * 0.01;
    }
  }
}

}  // namespace rhoban_utils
