#pragma once

#include <vector>
#include <map>
#include <string>
#include <json/json.h>

namespace rhoban_utils
{
class Function
{
public:
  Function();

  /**
   * Add a point (x, y) to the function
   */
  void addPoint(double x, double y);

  /**
   * Gets the xMax of the function
   */
  double getXMax();

  /**
   * Get the value for the given x
   */
  double get(double x, double smooth = -1.0);

  /**
   * Gets the value for the given x, modulo the size of the function
   */
  double getMod(double x);

  /**
   * Clear the spline
   */
  void clear();

  /**
   * Load a JSON file
   */
  static std::map<std::string, Function> fromFile(std::string filename);
  static std::map<std::string, Function> fromFile(std::string filename, Json::Value& json);

  /**
   * Write a JSON file
   */
  static void toFile(std::map<std::string, Function>& splines, std::string filename);

protected:
  /**
   * Function points
   */
  std::vector<double> points_x;
  std::vector<double> points_y;

  /**
   * The nth item of this array is the slope between the nth point and the
   * n+1th point
   */
  std::vector<double> ds;

  /**
   * Number of points
   */
  int nbPoints;
};

}  // namespace rhoban_utils
