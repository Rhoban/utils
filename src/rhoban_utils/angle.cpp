#define _USE_MATH_DEFINES
#include <cmath>

#include "rhoban_utils/angle.h"

#define EPSILON 0.000001

using namespace std;

namespace rhoban_utils
{
Angle::Angle(double a, Type type)
{
  double _a = a;
  if (type == RAD)
  {
    _a = rad2deg(_a);
  }
  value = fmod(_a, 360.0);
  if (value < 0)
  {
    value += 360.0;
  }
}

double Angle::getValue() const
{
  return value;
}

double Angle::getSignedValue() const
{
  if (value > 180)
  {
    return value - 360;
  }
  return value;
}

double Angle::getSignedValueRad() const
{
  return deg2rad(getSignedValue());
}

Angle Angle::fromXY(double x, double y)
{
  double radAngle = atan2(y, x);
  return Angle(radAngle * 180.0 / M_PI);
}
Angle Angle::arcsin(double x)
{
  return Angle(rad2deg(std::asin(x)));
}

Angle Angle::arccos(double x)
{
  return Angle(rad2deg(std::acos(x)));
}

Angle Angle::weightedAverage(const Angle& a1, double w1, const Angle& a2, double w2)
{
  double x = w1 * cos(a1) + w2 * cos(a2);
  double y = w1 * sin(a1) + w2 * sin(a2);
  return Angle(rad2deg(atan2(y, x)));
}

// Mean and stdDev from wikipedia
// http://en.wikipedia.org/wiki/Directional_statistics#The_fundamental_difference_between_linear_and_circular_statistics

Angle Angle::mean(const std::vector<Angle>& angles)
{
  double unused;
  return mean(angles, &unused);
}

Angle Angle::mean(const std::vector<Angle>& angles, double* stdDev)
{
  double sumCos(0), sumSin(0);
  for (unsigned int i = 0; i < angles.size(); i++)
  {
    sumCos += cos(angles[i]);
    sumSin += sin(angles[i]);
  }
  double meanCos, meanSin;
  meanCos = sumCos / angles.size();
  meanSin = sumSin / angles.size();
  Angle mean = fromXY(meanCos, meanSin);
  double norm = sqrt(meanCos * meanCos + meanSin * meanSin);
  if (norm >= 1)
  {  // Avoiding floating point exceptions
    *stdDev = 0;
  }
  else
  {
    *stdDev = sqrt(-2 * log(norm)) * 180 / M_PI;
  }
  return mean;
}

double Angle::stdDev(const std::vector<Angle>& angles)
{
  double stdDev;
  mean(angles, &stdDev);
  return stdDev;
}

bool Angle::equals(const Angle& other) const
{
  double delta = abs((*this - other).getSignedValue());
  return delta < EPSILON;
}

bool Angle::operator==(const Angle& a) const
{
  return a.getValue() == getValue();
}

Angle Angle::operator-(const Angle& a) const
{
  return Angle(value - a.value);
}

Angle Angle::operator+(const Angle& a) const
{
  return Angle(value + a.value);
}

double cos(const Angle& a)
{
  return std::cos(a.getValue() * M_PI / 180.0);
}

double sin(const Angle& a)
{
  return std::sin(a.getValue() * M_PI / 180.0);
}

double tan(const Angle& a)
{
  return std::tan(a.getValue() * M_PI / 180.0);
}

double deg2rad(double degreeAngle)
{
  return M_PI * degreeAngle / 180.0;
}

double rad2deg(double radAngle)
{
  return radAngle * 180.0 / M_PI;
}

Angle operator-(const Angle& a)
{
  return Angle(-a.getSignedValue());
}

Angle operator*(double x, const Angle& a)
{
  return Angle(x * a.getSignedValue());
}

Angle operator*(const Angle& a, double x)
{
  return x * a;
}

Angle operator/(const Angle& a, double x)
{
  return Angle(a.getSignedValue() / x);
}

ostream& operator<<(ostream& out, const Angle& a)
{
  return out << a.getValue();
}

double normalizeRad(double angleRad)
{
  double value = fmod(angleRad, 2 * M_PI);  // Bound in [-2*pi, 2*pi]
  if (value < -M_PI)
  {
    value += 2 * M_PI;
  }
  else if (value > M_PI)
  {
    value -= 2 * M_PI;
  }
  return value;
}

template <>
Angle getJsonVal<Angle>(const Json::Value& v)
{
  double val = getJsonVal<double>(v);
  return Angle(val);
}
}  // namespace rhoban_utils
