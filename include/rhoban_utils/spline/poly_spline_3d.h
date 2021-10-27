#pragma once

#include <Eigen/Dense>
#include <vector>
#include <algorithm>
#include "poly_spline.h"

namespace rhoban_utils
{
/**
 * PolySpline3D
 *
 * This is the same as poly spline, but using 3D points (Eigen::Vector3d)
 * Internally uses 3 polysplines
 */
class PolySpline3D
{
public:
  /**
   * Add a point with its x position, y value and
   * its derivative slope
   */
  void addPoint(double pos, Eigen::Vector3d val, Eigen::Vector3d delta);

  void clear();

  /**
   * Spline duration
   */
  double duration() const;

  /**
   * Return the spline interpolation
   * for given x position
   */
  Eigen::Vector3d get(double x) const;

  /**
   * Returns the spline speed interpolation
   * for given x position
   */
  Eigen::Vector3d getVel(double x) const;

  /**
   * Return the spline interpolation value
   * with x bound between 0 and 1
   */
  Eigen::Vector3d getMod(double x) const;

private:
  rhoban_utils::PolySpline xSpline;
  rhoban_utils::PolySpline ySpline;
  rhoban_utils::PolySpline zSpline;
};

}  // namespace rhoban_utils
