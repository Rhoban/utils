#include <stdio.h>
#include <errno.h>
#include "rhoban_utils/threading/priority.h"

namespace rhoban_utils
{
void set_thread_priority(int priority)
{
  struct sched_param param;
  int policy;
  pthread_getschedparam(pthread_self(), &policy, &param);
  param.sched_priority = priority;
  int r = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
  if (r != 0)
  {
    perror("set_thread_priority");
  }
}
}  // namespace rhoban_utils