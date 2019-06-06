#include <fstream>
#include <math.h>
#include <stdexcept>
#include <iostream>
// #include <boost/math/distributions/chi_squared.hpp>

#include "rhoban_utils/nominal/nominal.h"
#include "rhoban_utils/serialization/json_serializable.h"

namespace rhoban_utils
{
double chi2_cdf(double x, double k)
{
  (void)k;
  return sqrt(x);
  /*
  boost::math::chi_squared mydist(k);
  return boost::math::cdf(mydist, x);
  */
}

Json::Value eigenToJson(Eigen::VectorXd e)
{
  Json::Value json(Json::arrayValue);

  for (int k = 0; k < e.size(); k++)
  {
    json[k] = e[k];
  }

  return json;
}

Eigen::VectorXd jsonToEigenVector(Json::Value j)
{
  Eigen::VectorXd v(j.size());
  int k = 0;
  for (auto n : j)
  {
    v[k++] = n.asDouble();
  }

  return v;
}

Json::Value eigenToJson(Eigen::MatrixXd e)
{
  Json::Value json(Json::arrayValue);

  int k = 0;
  for (int r = 0; r < e.rows(); r++)
  {
    for (int c = 0; c < e.cols(); c++)
    {
      json[k++] = e(r, c);
    }
  }

  return json;
}

Eigen::MatrixXd jsonToEigenMatrix(Json::Value v)
{
  int s = sqrt(v.size());
  Eigen::MatrixXd m(s, s);

  int k = 0;
  for (int r = 0; r < s; r++)
  {
    for (int c = 0; c < s; c++)
    {
      m(r, c) = v[k++].asDouble();
    }
  }

  return m;
}

void Nominal::Bin::add(Eigen::VectorXd v)
{
  values.push_back(v);
}

void Nominal::Bin::compile(int dimensions)
{
  if (values.size() <= 1)
  {
    throw std::logic_error("A nominal bin is empty");
  }

  // Updating mean
  mean = Eigen::VectorXd(dimensions);
  for (int k = 0; k < dimensions; k++)
  {
    mean[k] = 0;
  }
  for (auto v : values)
  {
    mean += v;
  }
  mean /= (double)values.size();

  // Updating cov
  cov = Eigen::MatrixXd(dimensions, dimensions);
  for (int k = 0; k < dimensions; k++)
  {
    for (int l = 0; l < dimensions; l++)
    {
      cov(k, l) = 0;
    }
  }
  for (auto v : values)
  {
    Eigen::VectorXd d = v - mean;
    cov = cov + d * d.transpose();
  }
  cov = cov / ((double)(values.size() - 1));

  // Adding epsilon random to be sure the matrix is numerically inversible
  Eigen::MatrixXd epsilon = Eigen::MatrixXd::Random(cov.rows(), cov.cols());
  cov += epsilon * 1e-6;

  icov = cov.inverse();
}

Nominal::Nominal(int dimensions, int nbins) : dimensions(dimensions), nbins(nbins)
{
  for (int k = 0; k < nbins; k++)
  {
    bins[k] = Bin();
  }
}

Nominal::Bin& Nominal::getBin(double t)
{
  int bin = floor(nbins * t);

  return bins[bin];
}

void Nominal::add(double t, Eigen::VectorXd v)
{
  if (v.size() != dimensions)
  {
    throw std::logic_error("bad dimensions");
  }

  getBin(t).add(v);
}

double Nominal::Bin::chi2(Eigen::VectorXd v)
{
  Eigen::VectorXd tmp;
  tmp = v - mean;
  double chi2 = (icov * tmp).dot(tmp);
  return chi2;
}

double Nominal::Bin::score(Eigen::VectorXd v)
{
  return chi2_cdf(chi2(v), v.size());
}

double Nominal::guessT(Eigen::VectorXd v)
{
  double bestScore = std::numeric_limits<double>::max();
  double t = 0;

  for (auto bin : bins)
  {
    double score = bin.second.chi2(v);
    if (score < bestScore)
    {
      t = bin.first / (double)nbins;
      bestScore = score;
    }
  }

  return t;
}

void Nominal::compile()
{
  for (auto& entry : bins)
  {
    std::cout << "BIN " << entry.first << " : " << entry.second.values.size() << std::endl;
    fflush(stdout);
    entry.second.compile(dimensions);
  }
}

void Nominal::saveToJson(std::string filename)
{
  Json::Value json(Json::arrayValue);
  Json::StyledWriter writer;

  int k = 0;
  for (auto entry : bins)
  {
    auto bin = entry.second;
    Json::Value b(Json::arrayValue);
    b[0] = eigenToJson(bin.mean);
    b[1] = eigenToJson(bin.cov);
    b[2] = eigenToJson(bin.icov);
    json[k++] = b;
  }

  std::ofstream of(filename);
  of << writer.write(json);
  of.close();
}

void Nominal::loadJson(std::string filename)
{
  Json::Value json;
  try
  {
    json = rhoban_utils::file2Json(filename);
  }
  catch (const rhoban_utils::JsonParsingError& exc)
  {
    std::cerr << "Nominal::loadJson: Failed to load from '" << filename << "' :" << exc.what() << std::endl;
  }

  if (!json.isNull())
  {
    int k = 0;
    for (auto e : json)
    {
      bins[k].values.clear();
      bins[k].mean = jsonToEigenVector(e[0]);
      bins[k].cov = jsonToEigenMatrix(e[1]);
      bins[k].icov = jsonToEigenMatrix(e[2]);
      k++;
    }
  }
}

}  // namespace rhoban_utils
