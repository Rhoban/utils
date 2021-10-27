#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include "rhoban_utils/spline/poly_spline.h"

namespace rhoban_utils
{
void PolySpline::addPoint(double pos, double val, double delta, double ddelta)
{
  struct Point point = { pos, val, delta, ddelta };
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
        double xi = (x - _splines[i].min) / (_splines[i].max - _splines[i].min);
        if (type == Value)
        {
          return polynomValue(xi, _splines[i].poly);
        }
        else if (type == Speed)
        {
          return polynomDiff(xi, _splines[i].poly);
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
  return p.f + t * (t * (t * (t * (t * p.a + p.b) + p.c) + p.d) + p.e);
}

double PolySpline::polynomDiff(double t, const Polynom& p)
{
  return t * (t * (t * (t * 5 * p.a + 4 * p.b) + 3 * p.c) + 2 * p.d) + p.e;
}

PolySpline::Polynom PolySpline::polynomFit(double f0, double fd0, double fdd0, double f1, double fd1, double fdd1)
{
  struct PolySpline::Polynom polynom = { -6 * f0 + 6 * f1 - 3 * fd0 - 3 * fd1 - fdd0 / 2 + fdd1 / 2,
                                         15 * f0 - 15 * f1 + 8 * fd0 + 7 * fd1 + 3 * fdd0 / 2 - fdd1,
                                         -10 * f0 + 10 * f1 - 6 * fd0 - 4 * fd1 - 3 * fdd0 / 2 + fdd1 / 2,
                                         fdd0 / 2,
                                         fd0,
                                         f0 };

  return polynom;
}  // namespace rhoban_utils

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
    struct Spline spline = { polynomFit(_points[i - 1].value, _points[i - 1].delta, _points[i - 1].ddelta,
                                        _points[i].value, _points[i].delta, _points[i].ddelta),
                             _points[i - 1].position, _points[i].position };

    _splines.push_back(spline);
  }
}

}  // namespace rhoban_utils
