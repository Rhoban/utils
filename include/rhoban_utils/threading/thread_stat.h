#pragma once

#include <unistd.h>
#include <sys/resource.h>

#include <chrono>
#include <deque>
#include <string>
#include <rhoban_utils/timing/time_stamp.h>

namespace rhoban_utils
{

class ThreadStat
{
public:
    struct CpuUsage
    {
        double userPercent   = 0.0;
        double systemPercent = 0.0;
        double totalPercent  = 0.0;
    };

    ThreadStat(const std::string &name);

    pid_t getThreadId() const;

    CpuUsage update() ;


    void reset() ;

private:
    std::string asStr(CpuUsage usage);

    using Clock = std::chrono::steady_clock;

    struct Sample
    {
        Clock::time_point wallTime;
        double userCpu;
        double sysCpu;
    };

    static double timevalToSeconds(const timeval& tv);

private:
    pid_t m_threadId;
    std::deque<Sample> m_samples;
    std::string name;
    double msg_period=-1.0;
    TimeStamp lastMsg;
};

} // namespace rhoban_utils