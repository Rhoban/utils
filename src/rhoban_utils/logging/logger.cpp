#include <cstdio>
#include <time.h>
#include <cstdarg>
#include <cmath>
#ifndef WIN32
#include <unistd.h>
#include <sys/time.h>
#endif

#include "rhoban_utils/logging/logger.h"

#define T_COLOR_RED "\x1b[0;31m"
#define T_COLOR_GREEN "\x1b[0;32m"
#define T_COLOR_YELLOW "\x1b[0;33m"
#define T_COLOR_BLUE "\x1b[0;34m"
#define T_COLOR_RESET "\x1b[0m"

namespace rhoban_utils
{
Logger::Logger(std::string module_, LoggerLevel level_) : module(module_), level(level_)
{
#ifdef WIN32
  colors = true;
#else
  colors = true;  // isatty(STDERR_FILENO);
#endif
}

void Logger::getTime(uint8_t& hour, uint8_t& min, uint8_t& sec, uint16_t& ms)
{
#ifndef WIN32
  struct timeval tv;

  gettimeofday(&tv, NULL);

  ms = std::lrint(tv.tv_usec / 1000.0);  // Round to nearest millisec
  if (ms >= 1000)
  {  // Allow for rounding up to nearest second
    ms -= 1000;
    tv.tv_sec++;
  }

  struct tm tm_info;
  localtime_r(&tv.tv_sec, &tm_info);

  hour = tm_info.tm_hour;
  min = tm_info.tm_min;
  sec = tm_info.tm_sec;
#endif
}

void Logger::prefix()
{
#ifndef WIN32
  uint8_t hour, min, sec;
  uint16_t ms;
  getTime(hour, min, sec, ms);
  fprintf(stderr, "[%02d:%02d:%02d:%03d] ", hour, min, sec, ms);
#endif
  fprintf(stderr, "[%s] ", module.c_str());
}

void Logger::colorize(LoggerLevel lvl)
{
  if (colors)
  {
    const char* color = NULL;
    switch (lvl)
    {
      case LoggerError:
        color = T_COLOR_RED;
        break;
      case LoggerWarning:
        color = T_COLOR_YELLOW;
        break;
      case LoggerMessage:
        color = T_COLOR_RESET;
        break;
      case LoggerSuccess:
        color = T_COLOR_GREEN;
        break;
      case LoggerDebug:
        color = T_COLOR_BLUE;
        break;
      case LoggerNone:
        break;
    }

    if (color != NULL)
    {
      fprintf(stderr, "%s", color);
    }
  }
}

void Logger::resetColor()
{
  if (colors)
  {
    fprintf(stderr, "%s", T_COLOR_RESET);
  }
}

#define LOG(lvl)                                                                                                       \
  if (lvl <= level)                                                                                                    \
  {                                                                                                                    \
    colorize(lvl);                                                                                                     \
    prefix();                                                                                                          \
    va_list args;                                                                                                      \
    va_start(args, fmt);                                                                                               \
    vfprintf(stderr, fmt, args);                                                                                       \
    va_end(args);                                                                                                      \
    fprintf(stderr, "\n");                                                                                             \
    resetColor();                                                                                                      \
  }

void Logger::error(const char* fmt, ...)
{
  LOG(LoggerError);
}

void Logger::warning(const char* fmt, ...)
{
  LOG(LoggerWarning);
}

void Logger::log(const char* fmt, ...)
{
  LOG(LoggerMessage);
}

void Logger::success(const char* fmt, ...)
{
  LOG(LoggerSuccess);
}

void Logger::debug(const char* fmt, ...)
{
  LOG(LoggerDebug);
}
}  // namespace rhoban_utils
