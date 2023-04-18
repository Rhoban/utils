#pragma once

#include <vector>
#include <algorithm>

namespace rhoban_utils
{
/**
 * PolySpline
 *
 * Splines primitive based on third order polynoms
 * for smooth and differentiable adjunction
 */
class PolySpline
{
public:
  struct Point
  {
    double position;
    double value;
    double delta;
  };

  typedef std::vector<Point> Points;

  /**
   * Spline duration
   */
  double duration() const;

  /**
   * Add a point with its x position, y value and
   * its derivative slope
   */
  void addPoint(double pos, double val, double delta, bool angle_spline = false);

  void clear();

  /**
   * Return the spline interpolation
   * for given x position
   */
  double get(double x);

  /**
   * Returns the spline speed interpolation
   * for given x position
   */
  double getVel(double x);

  enum ValueType
  {
    Value,
    Speed
  };
  double interpolation(double x, ValueType type);

  /**
   * Return the spline interpolation value
   * with x bound between 0 and 1
   */
  double getMod(double x);

  /**
   * Access to internal Points container
   */
  const Points& points() const;

private:
  bool dirty = true;

  struct Polynom
  {
    double a;
    double b;
    double c;
    double d;
  };

  struct Spline
  {
    Polynom poly;
    double min;
    double max;
  };

  typedef std::vector<Spline> Splines;

  /**
   * Spline Points container
   */
  Points _points;

  /**
   * Splines container
   */
  Splines _splines;

  /**
   * Fast exponentation to compute
   * given polynom value
   */
  static double polynomValue(double t, const Polynom& p);

  /**
   * Polynom diff. value
   */
  static double polynomDiff(double t, const Polynom& p);

  /**
   * Fit a polynom between 0 and 1 with
   * given value and slope
   */
  static Polynom polynomFit(double t1, double val1, double delta1, double t2, double val2, double delta2);

  /**
   * Recompute splines interpolation model
   */
  void computeSplines();
};

}  // namespace rhoban_utils
