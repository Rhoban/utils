#pragma once

#include <vector>

namespace rhoban_utils
{
double average(const std::vector<double>& values);

double variance(const std::vector<double>& values, double* avg = nullptr);

double standardDeviation(const std::vector<double>& values, double* avg = nullptr);

}  // namespace rhoban_utils
