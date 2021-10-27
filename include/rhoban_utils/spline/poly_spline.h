#pragma once

#include <vector>
#include <algorithm>

namespace rhoban_utils
{
/**
 * PolySpline
 *
 * Quintic splines (can be used as cubic or linear as well)
 */
class PolySpline
{
public:
  struct Point
  {
    double position;
    double value;
    double delta;
    double ddelta;
  };

  typedef std::vector<Point> Points;

  /**
   * Add a point with its x position, y value and
   * its derivative slope
   */
  void addPoint(double pos, double val, double delta=0., double ddelta = 0.);

  void clear();

  /**
   * Return the spline interpolation
   * for given x position
   */
  double get(double x) const;

  /**
   * Returns the spline speed interpolation
   * for given x position
   */
  double getVel(double x) const;

  enum ValueType
  {
    Value,
    Speed
  };
  double interpolation(double x, ValueType type) const;

  /**
   * Return the spline interpolation value
   * with x bound between 0 and 1
   */
  double getMod(double x) const;

  /**
   * Access to internal Points container
   */
  const Points& points() const;

private:
  struct Polynom
  {
    double a;
    double b;
    double c;
    double d;
    double e;
    double f;
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
  static Polynom polynomFit(double f0, double fd0, double fdd0, double f1, double fd1, double fdd1);

  /**
   * Recompute splines interpolation model
   */
  void computeSplines();
};

}  // namespace rhoban_utils
