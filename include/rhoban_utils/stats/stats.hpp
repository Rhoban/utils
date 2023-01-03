#pragma once

#include <cmath>
#include <vector>

namespace rhoban_utils
{
template <typename T>
double average(const T& values)
{
  if (values.size())
  {
    double result = 0;

    for (auto& v : values)
    {
      result += v;
    }

    return result / values.size();
  }
  else
  {
    return 0;
  }
}

template <typename T>
double variance(const T& values, double* avg_ = nullptr)
{
  double avg = average(values);

  if (avg_ != nullptr)
  {
    *avg_ = avg;
  }

  if (values.size())
  {
    double result = 0;

    for (auto& v : values)
    {
      result += pow(v - avg, 2);
    }

    return result / values.size();
  }
  else
  {
    return 0;
  }
}

template <typename T>
double standardDeviation(const T& values, double* avg = nullptr)
{
  return sqrt(variance(values, avg));
}

}  // namespace rhoban_utils
