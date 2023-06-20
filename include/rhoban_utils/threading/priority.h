#pragma once

#include <pthread.h>
#include <sys/resource.h>
#include <sys/time.h>

namespace rhoban_utils
{
/**
 * @brief This switches the current (p)thread scheduling to FIFO with the given priority
 */
void set_thread_priority(int priority);
}  // namespace rhoban_utils