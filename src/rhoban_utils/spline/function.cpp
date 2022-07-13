#include <fstream>
#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <json/json.h>

#include "rhoban_utils/util.h"
#include "rhoban_utils/spline/function.h"
#include "rhoban_utils/serialization/json_serializable.h"

namespace rhoban_utils
{
Function::Function() : nbPoints(0)
{
}

void Function::clear()
{
  points_x.clear();
  points_y.clear();
  ds.clear();
  nbPoints = 0;
}

std::map<std::string, Function> Function::fromFile(std::string filename)
{
  Json::Value json;
  fromFile(filename, json);
}

std::map<std::string, Function> Function::fromFile(std::string filename, Json::Value& json)
{
  // Reading json content
  std::map<std::string, Function> result;
  json = rhoban_utils::file2Json(filename);
  // Interpreting Json object
  if (json.isObject())
  {
    for (auto name : json.getMemberNames())
    {
      auto data = json[name];
      Function f;
      if (data.isArray())
      {
        for (unsigned int k = 0; k < data.size(); k++)
        {
          auto entry = data[k];
          if (entry.isArray() && entry.size() == 2)
          {
            f.addPoint(entry[0].asDouble(), entry[1].asDouble());
          }
        }
      }
      result[name] = f;
    }
  }
  else if (json.isArray())
  {
    // fprintf(stderr,"START loading time based json file\n");
    for (unsigned int k = 0; k < json.size(); k++)
    {
      auto entry = json[k];
      if (entry.isObject())
      {
        std::map<std::string, double> settings;
        for (auto name : entry.getMemberNames())
        {
          auto value = entry[name];
          settings[name] = value.asDouble();
          // fprintf(stderr,"\t building settings: %s =>  %lf \n",name.c_str(),settings[name]);
        }
        double time = settings["time"];
        // fprintf(stderr,"\ttime is %lf \n",time);
        for (auto v : settings)
        {
          if (v.first != "time")
          {
            if (result.find(v.first) == result.end())
            {
              result[v.first] = Function();
            }
            result[v.first].addPoint(time, v.second);
            // fprintf(stderr,"\tadd entry for %s : %lf = %lf \n",v.first.c_str(),time,v.second);
          }
        }
      }
    }
    // fprintf(stderr,"END loading time based json file\n");
  }

  // for(auto e : result){
  // fprintf(stderr,"entries for %s : \n",e.first.c_str());
  // for(int i=0;i<e.second.points_x.size();++i){
  // fprintf(stderr,"\t%lf => %lf\n",e.second.points_x[i],e.second.points_y[i]);
  //}
  //}
  return result;
}

void Function::toFile(std::map<std::string, Function>& splines, std::string filename)
{
  Json::Value json(Json::objectValue);
  Json::StyledWriter writer;

  for (auto& entry : splines)
  {
    auto& key = entry.first;
    auto& spline = entry.second;
    json[key] = Json::Value(Json::arrayValue);
    for (int k = 0; k < spline.nbPoints; k++)
    {
      Json::Value pt(Json::arrayValue);
      pt[0] = spline.points_x[k];
      pt[1] = spline.points_y[k];
      json[key][k] = pt;
    }
  }

  std::string data;
  data = writer.write(json);
  file_put_contents(filename, data);
}

double Function::getXMax()
{
  if (nbPoints == 0)
  {
    return 0.0;
  }

  return points_x[nbPoints - 1];
}

void Function::addPoint(double x, double y)
{
  points_x.push_back(x);
  points_y.push_back(y);

  if (nbPoints > 0)
  {
    float d = points_y[nbPoints] - points_y[nbPoints - 1];
    d /= points_x[nbPoints] - points_x[nbPoints - 1];
    ds.push_back(d);
  }

  nbPoints++;
}

double Function::get(double x, double smooth)
{
  if (nbPoints == 0)
  {
    return 0.0;
  }

  if (points_x[0] >= x)
  {
    return points_y[0];
  }
  if (points_x[nbPoints - 1] <= x)
  {
    return points_y[nbPoints - 1];
  }

  int a = 0, b = nbPoints - 1;
  int i = (a + b) / 2;
  while (a != b)
  {
    if (points_x[i] >= x)
    {
      if (i == 0 || points_x[i - 1] <= x)
      {
        a = b;
      }
      else
      {
        b = i;
        i = (a + b) / 2;
      }
    }
    else
    {
      if (a == i)
      {
        a++;
      }
      else
      {
        a = i;
      }
      i = (a + b) / 2;
    }
  }

  if (smooth <= 0.0)
    return points_y[i - 1] + ds[i - 1] * (x - points_x[i - 1]);

  // x is between points_x[i-1] and points_x[i]
  double linear = points_y[i - 1] + ds[i - 1] * (x - points_x[i - 1]);
  double left = x - points_x[i - 1];
  double right = points_x[i] - x;
  if ((right < smooth) and (right < left))
  {
    double d = (points_x[i] - x) / smooth;
    return (1 - d) * points_y[i] + d * linear;
  }
  if (left < smooth)
  {
    double d = (x - points_x[i - 1]) / smooth;
    return (1 - d) * points_y[i - 1] + d * linear;
  }
  return linear;
}

double Function::getMod(double x)
{
  double maxX = getXMax();

  if (x < 0.0 || x > maxX)
  {
    x -= maxX * (int)(x / maxX);
  }

  return get(x);
}

}  // namespace rhoban_utils
