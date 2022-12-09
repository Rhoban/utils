#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>
#include "rhoban_utils/spline/poly_spline.h"

namespace rhoban_utils
{
void PolySpline::addPoint(double pos, double val, double delta)
{
  struct Point point = { pos, val, delta };
  if (_points.size() > 0 && pos <= _points.back().position)
  {
    throw std::runtime_error("Trying to add a point in a cublic spline before a previous one");
  }
  _points.push_back(point);
  computeSplines();
}

void PolySpline::clear()
{
  _points.clear();
  _splines.clear();
}

double PolySpline::duration() const
{
  return _splines[_splines.size() - 1].max - _splines[0].min;
}

/**
 * Return the spline interpolation
 * for given x position
 */
double PolySpline::interpolation(double x, PolySpline::ValueType type) const
{
  if (_points.size() == 0)
  {
    return 0.0;
  }
  else if (_points.size() == 1)
  {
    if (type == Value)
    {
      return _points.front().value;
    }
    else
    {
      return _points.front().delta;
    }
  }
  else
  {
    if (x < _splines.front().min)
    {
      x = _splines.front().min;
    }
    if (x > _splines.back().max)
    {
      x = _splines.back().max;
    }

    for (size_t i = 0; i < _splines.size(); i++)
    {
      if (x >= _splines[i].min && x <= _splines[i].max)
      {
        if (type == Value)
        {
          return polynomValue(x, _splines[i].poly);
        }
        else if (type == Speed)
        {
          return polynomDiff(x, _splines[i].poly);
        }
      }
    }
    return 0.0;
  }
}

double PolySpline::get(double x) const
{
  return interpolation(x, Value);
}

double PolySpline::getVel(double x) const
{
  return interpolation(x, Speed);
}

/**
 * Access to internal Points container
 */
const PolySpline::Points& PolySpline::points() const
{
  return _points;
}

double PolySpline::polynomValue(double t, const Polynom& p)
{
  return p.d + t * (t * (p.a * t + p.b) + p.c);
}

double PolySpline::polynomDiff(double t, const Polynom& p)
{
  return t * (3 * p.a * t + 2 * p.b) + p.c;
}

PolySpline::Polynom PolySpline::polynomFit(double t1, double val1, double delta1, double t2, double val2, double delta2)
{
  Eigen::Matrix4d M;
  double t1_2 = t1 * t1;
  double t1_3 = t1_2 * t1;
  double t2_2 = t2 * t2;
  double t2_3 = t2_2 * t2;

  M << t1_3, t1_2, t1, 1,      //
      3 * t1_2, 2 * t1, 1, 0,  //
      t2_3, t2_2, t2, 1,       //
      3 * t2_2, 2 * t2, 1, 0   //
      ;

  Eigen::Vector4d v;
  v << val1, delta1, val2, delta2;

  Eigen::Vector4d abcd = M.inverse() * v;

  struct PolySpline::Polynom polynom = { abcd[0], abcd[1], abcd[2], abcd[3] };

  return polynom;
}

/**
 * Return the spline interpolation value
 * with x bound between 0 and 1
 */
double PolySpline::getMod(double x) const
{
  if (x < 0.0)
  {
    x = 1.0 + (x - ((int)x / 1));
  }
  else if (x > 1.0)
  {
    x = (x - ((int)x / 1));
  }

  return get(x);
}

void PolySpline::computeSplines()
{
  _splines.clear();
  if (_points.size() < 2)
  {
    return;
  }

  /*
     std::sort(
     _points.begin(),
     _points.end(),
     [](const Point& p1, const Point& p2) -> bool {
     return p1.position < p2.position;
     });
     */

  for (size_t i = 1; i < _points.size(); i++)
  {
    if (fabs(_points[i - 1].position - _points[i].position) < 0.00001)
    {
      continue;
    }
    struct Spline spline = { polynomFit(_points[i - 1].position, _points[i - 1].value, _points[i - 1].delta,
                                        _points[i].position, _points[i].value, _points[i].delta),
                             _points[i - 1].position, _points[i].position };

    _splines.push_back(spline);
  }
}

}  // namespace rhoban_utils
