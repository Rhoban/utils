#pragma once

#include <vector>

namespace rhoban_utils
{

double average(std::vector<double> values);

double variance(std::vector<double> values, double *avg = nullptr);

double standardDeviation(std::vector<double> values, double *avg = nullptr);

}