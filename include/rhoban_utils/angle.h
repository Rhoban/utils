#pragma once

#include <vector>
#include <ostream>

#include "serialization/json_serializable.h"

namespace rhoban_utils
{
/* This class aims to handle most of the frequent problems when manipulating
 * angles. In this class, if not specified, values are always between 0 and
 * 360.
 */
class Angle
{
protected:
  double value;

public:
  Angle() : value(0.0){};

  enum Type
  {
    DEG,
    RAD
  };

  Angle(double a, Type type = DEG);
  Angle(const Angle& other) : value(other.value){};

  /* Value in [0, 360] */
  double getValue() const;
  /* Value in [-180,180] */
  double getSignedValue() const;
  /// Value in [-pi,pi]
  double getSignedValueRad() const;

  static Angle fromXY(double x, double y);
  static Angle mean(const std::vector<Angle>& angles);
  static Angle mean(const std::vector<Angle>& angles, double* stdDev);
  static double stdDev(const std::vector<Angle>& angles);

  static Angle arcsin(double x);
  static Angle arccos(double x);
  static Angle weightedAverage(const Angle& a1, double w1, const Angle& a2, double w2);

  bool equals(const Angle& other) const;

  Angle operator-(const Angle& a) const;
  Angle operator+(const Angle& a) const;

  bool operator==(const Angle& a) const;
};

Angle operator-(const Angle& a);
Angle operator*(double x, const Angle& a);
Angle operator*(const Angle& a, double x);
Angle operator/(const Angle& a, double x);

std::ostream& operator<<(std::ostream& out, const Angle& a);

double sin(const Angle& a);
double cos(const Angle& a);
double tan(const Angle& a);

double deg2rad(double degreeAngle);
double rad2deg(double radAngle);

/// return the normalized angle in [-pi,pi] from radAngle
double normalizeRad(double radAngle);

/// Reading an Angle from a JsonValue
template <>
Angle getJsonVal<Angle>(const Json::Value& v);

}  // namespace rhoban_utils
