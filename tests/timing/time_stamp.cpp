#include <gtest/gtest.h>
#include <stdlib.h>
#include "rhoban_utils/timing/time_stamp.h"
#include <sys/timeb.h>
#include <time.h>
#include <ctime>
#include <ratio>
#include <chrono>
#include <string>
using namespace rhoban_utils;
using namespace std::chrono;

// test getTimeMS
TEST(time_stamp, testTime_stamp1)
{
  std::chrono::steady_clock::time_point timeValP1 = std::chrono::steady_clock::now();
  TimeStamp* time_stamp = new TimeStamp(std::chrono::steady_clock::now());
  TimeStamp* time_stamp1 = new TimeStamp();

  // time_stamp1 should have time =0
  EXPECT_EQ(time_stamp1->getTimeMS(), 0);

  // time_stamp should have time > 0
  EXPECT_GT(time_stamp->getTimeMS(), 0);

  std::chrono::steady_clock::time_point timeValP2 = std::chrono::steady_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(timeValP2 - timeValP1);

  // time_stamp should be  > 0 time_span
  EXPECT_GT(time_stamp->getTimeMS(), time_span.count());
}

// test diffSec and diffMs between two time stamp
TEST(time_stamp, timeDiff)
{
  int wait_time_us = 5000;
  TimeStamp time_stamp1 = TimeStamp::now();
  usleep(wait_time_us);
  TimeStamp time_stamp2 = TimeStamp::now();

  for (bool utc : { true, false })
  {
    double measured_sec = diffSec(time_stamp1, time_stamp2, utc);
    double measured_ms = diffMs(time_stamp1, time_stamp2, utc);
    double expected_ms = wait_time_us / 1000.0;
    double expected_sec = expected_ms / 1000.0;
    double tol_ms = 1.0;

    EXPECT_NEAR(expected_ms, measured_ms, tol_ms);
    EXPECT_NEAR(expected_sec, measured_sec, tol_ms / 1000);
  }
}

TEST(time_stamp, fromMS)
{
  int ms_input = 50021;
  TimeStamp t = TimeStamp::fromMS(ms_input);
  EXPECT_FLOAT_EQ(ms_input, t.getTimeMS());
}

// test operator witch should return true if diffMs (time_stamp1 ,time_stamp2) > 0
// return false if diffMs (time_stamp1 ,time_stamp2) < 0
TEST(time_stamp, operator_m)
{
  const TimeStamp* time_stamp1 = new TimeStamp(std::chrono::steady_clock::now());
  const TimeStamp* time_stamp2 = new TimeStamp(std::chrono::steady_clock::now());

  bool valbool1 = operator<(*time_stamp1, *time_stamp2);
  bool valbool2 = operator>(*time_stamp1, *time_stamp2);

  // operator_> should be true
  EXPECT_TRUE(valbool1);
  // operator_< should be false
  EXPECT_FALSE(valbool2);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
