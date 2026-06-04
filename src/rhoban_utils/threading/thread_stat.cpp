#include "rhoban_utils/threading/thread_stat.h"
#include <iomanip>
#include <ios>
#include <sstream>
#include <iostream>
namespace rhoban_utils
{

    ThreadStat::ThreadStat(const std::string &name)
        : m_threadId(gettid()),name(name)
    {
        reset();
        const char *p=getenv("RHOBAN_THREAD_STAT");
        std::cout<<"ThreadStat for thread "<<name<<" initialized with thread id "<<m_threadId<<std::endl;
        if (p!=nullptr)
        {
            std::cout<<"ThreadStat for thread "<<name<<" will print every "<<p<<" seconds"<<std::endl;
        }
        else
        {
            std::cout<<"ThreadStat for thread "<<name<<" will not print automatically (set RHOBAN_THREAD_STAT environment variable to enable it)"<<std::endl;
        }
        if (p!=nullptr){
            msg_period=atof(p);
            std::cout<<"ThreadStat for thread "<<name<<" will print every "<<msg_period<<" seconds"<<std::endl;
        }
        this->name.resize(20, ' ');
        lastMsg = TimeStamp::now();
    }
    
    pid_t ThreadStat::getThreadId() const
    {
        return m_threadId;
    }


    ThreadStat::CpuUsage ThreadStat::update()
    {
        Sample current;

        struct rusage ru{};
        getrusage(RUSAGE_THREAD, &ru);

        current.wallTime = Clock::now();
        current.userCpu  = timevalToSeconds(ru.ru_utime);
        current.sysCpu   = timevalToSeconds(ru.ru_stime);

        m_samples.push_back(current);

        // Conserver uniquement les échantillons
        // couvrant approximativement la dernière seconde.
        while (m_samples.size() >= 2)
        {
            double age =
                std::chrono::duration<double>(
                    current.wallTime - m_samples.front().wallTime)
                    .count();

            if (age <= 1.0)
                break;

            m_samples.pop_front();
        }

        CpuUsage result;

        if (m_samples.size() >= 2)
        {
            const Sample& oldest = m_samples.front();

            double wall =
                std::chrono::duration<double>(
                    current.wallTime - oldest.wallTime)
                    .count();

            if (wall > 0.0)
            {
                const double userCpu =
                    current.userCpu - oldest.userCpu;

                const double sysCpu =
                    current.sysCpu - oldest.sysCpu;

                result.userPercent =
                    100.0 * userCpu / wall;

                result.systemPercent =
                    100.0 * sysCpu / wall;

                result.totalPercent =
                    result.userPercent +
                    result.systemPercent;
            }
        }

        //std::cout<<"msg period "<<msg_period<<" lastMsg age "<<diffSec(lastMsg,TimeStamp::now())<<std::endl;
        if ((msg_period > 0.0) && (diffSec(lastMsg,TimeStamp::now()) > msg_period))
        {
            std::cerr << asStr(result) << std::endl;
            lastMsg = TimeStamp::now();
        }
        return result;
    }

    std::string ThreadStat::asStr(CpuUsage usage) {
        std::stringstream ss;
        if (usage.totalPercent > 80.0)
        {
            ss << "\033[1;31m"; // Red and bold
        }
        else if (usage.totalPercent > 50.0)
        {
            ss << "\033[1;33m"; // Yellow and bold
        }
        else if (usage.totalPercent > 10.0)
        {
            ss << "\033[1;34m"; // Blue and bold
        }
        else
        {
            ss << "\033[1;32m"; // Green and bold
        }
        ss <<std::fixed<<std::setprecision(2)<< "Thread "<<name<<" [" << m_threadId << "] CPU usage: user " << usage.userPercent << "% / system " 
           << usage.systemPercent << "% / total "<< usage.totalPercent << "%";
        ss << "\033[0m"; // Reset color
        return ss.str();
    }


    void ThreadStat::reset()
    {
        m_samples.clear();

        struct rusage ru{};
        getrusage(RUSAGE_THREAD, &ru);

        Sample initial;

        initial.wallTime = Clock::now();
        initial.userCpu  = timevalToSeconds(ru.ru_utime);
        initial.sysCpu   = timevalToSeconds(ru.ru_stime);

        m_samples.push_back(initial);
    }


    double ThreadStat::timevalToSeconds(const timeval& tv)
    {
        return static_cast<double>(tv.tv_sec)
             + static_cast<double>(tv.tv_usec) * 1e-6;
    }

}