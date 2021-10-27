#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include "rhoban_utils/spline/poly_spline_3d.h"

namespace rhoban_utils
{
void PolySpline3D::clear()
{
  xSpline.clear();
  ySpline.clear();
  zSpline.clear();
}

double PolySpline3D::duration() const
{
  return xSpline.duration();
}

void PolySpline3D::addPoint(double pos, Eigen::Vector3d val, Eigen::Vector3d delta)
{
  xSpline.addPoint(pos, val[0], delta[0]);
  ySpline.addPoint(pos, val[1], delta[1]);
  zSpline.addPoint(pos, val[2], delta[2]);
}

Eigen::Vector3d PolySpline3D::get(double x) const
{
  return Eigen::Vector3d(xSpline.get(x), ySpline.get(x), zSpline.get(x));
}

Eigen::Vector3d PolySpline3D::getVel(double x) const
{
  return Eigen::Vector3d(xSpline.getVel(x), ySpline.getVel(x), zSpline.getVel(x));
}

Eigen::Vector3d PolySpline3D::getMod(double x) const
{
  return Eigen::Vector3d(xSpline.getMod(x), ySpline.getMod(x), zSpline.getMod(x));
}

}  // namespace rhoban_utils