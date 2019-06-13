#include "rhoban_utils/timing/benchmark.h"

#include "rhoban_utils/util.h"

#include <algorithm>
#include <exception>
#include <iomanip>
#include <sstream>
#include <tuple>
#include <vector>
#include <string>
#include <ctime>
#include <cstdlib>
#include <fstream>

using namespace std::chrono;

namespace rhoban_utils
{
/* Static variables */
Benchmark* Benchmark::current = NULL;

Benchmark::Benchmark(Benchmark* f, const std::string& n) : father(f), name(n), elapsedSec(0), nbIterations(0)
{
  startSession();
}

Benchmark* Benchmark::getCurrent()
{
  if (current == NULL)
    open("Default");
  return current;
}

Benchmark::~Benchmark()
{
  for (auto& c : children)
  {
    delete (c.second);
  }
}

void Benchmark::startSession()
{
  openingTime = steady_clock::now();
}

void Benchmark::endSession()
{
  closingTime = steady_clock::now();
  elapsedSec += diffSec(openingTime, closingTime);
  nbIterations++;
}

void Benchmark::open(const std::string& benchmarkName)
{
  // If child is not existing yet:
  Benchmark* childBenchmark = NULL;
  if (current == NULL || current->children.count(benchmarkName) == 0)
  {
    childBenchmark = new Benchmark(current, benchmarkName);
    if (current != NULL)
    {
      current->children[benchmarkName] = childBenchmark;
    }
  }
  else
  {
    childBenchmark = current->children.at(benchmarkName);
    childBenchmark->startSession();
  }
  current = childBenchmark;
}

double Benchmark::close(const char* expectedName, bool print, int detailLevel, std::ostream& out)
{
  if (current == NULL)
    throw std::runtime_error("No active benchmark to close");
  Benchmark* toClose = current;

  if (toClose->name != expectedName)
  {
    std::ostringstream oss;
    oss << "Invalid close of Benchmark:" << std::endl
        << "\tActive benchmark name: '" << toClose->name << "'" << std::endl
        << "\tExpected benchmark name: '" << expectedName << "'";
    throw std::runtime_error(oss.str());
  }
  return close(print, detailLevel, out);
}

double Benchmark::close(bool print, int detailLevel, std::ostream& out)
{
  if (current == NULL)
    throw std::runtime_error("No active benchmark to close");
  Benchmark* toClose = current;
  toClose->endSession();

  current = toClose->father;
  if (print)
  {
    toClose->print(out, detailLevel);
  }

  double elapsedTime = toClose->getTime();
  // Suppress Benchmark if the link is lost
  if (current == NULL)
  {
    delete (toClose);
  }
  return elapsedTime;
}

double Benchmark::closeUntil(const std::string& stopName)
{
  while (current != NULL && current->name != stopName)
  {
    close();
  }
  if (current == NULL)
  {
    throw std::runtime_error(DEBUG_INFO + " closed all benchmarks without finding a benchmark named: '" + stopName +
                             "'");
  }
  return close();
}

double Benchmark::getTime() const
{
  return elapsedSec;
}

double Benchmark::getSubTime() const
{
  double t = 0;
  for (auto& c : children)
  {
    t += c.second->getTime();
  }
  return t;
}

// Printable data: <name, elapsedTime, link>
typedef std::tuple<std::string, double, Benchmark*> printableEntry;
int cmp(const printableEntry& a, const printableEntry& b)
{
  return std::get<1>(a) < std::get<1>(b);
}

void Benchmark::print(std::ostream& out, int maxDepth)
{
  // Formatting specifically
  int precision = 3;
  int width = 8;
  int oldPrecision = out.precision();
  out.precision(precision);
  out.setf(std::ios::fixed, std::ios::floatfield);

  print(out, 0, width, maxDepth);

  // Restoring default
  out.precision(oldPrecision);
  out.unsetf(std::ios::floatfield);
}

void Benchmark::print(std::ostream& out, int depth, int width, int maxDepth)
{
  // Build and sort printable data
  typedef std::tuple<std::string, double, Benchmark*> printableEntry;
  std::vector<printableEntry> subFields;
  for (auto& c : children)
  {
    subFields.push_back(printableEntry(c.first, c.second->getTime(), c.second));
  }
  // Add Unknown field only if there are other information
  if (subFields.size() > 0)
  {
    subFields.push_back(printableEntry("Unknown", getTime() - getSubTime(), NULL));
  }
  // Ordering
  std::sort(subFields.begin(), subFields.end(), cmp);
  // Printing if allowed
  if (maxDepth < 0 || depth < maxDepth)
  {
    for (auto& f : subFields)
    {
      // For Benchmark, print them
      if (std::get<2>(f) != NULL)
      {
        std::get<2>(f)->print(out, depth + 1, width, maxDepth);
      }
      // Print entries
      else
      {
        for (int i = 0; i < depth + 1; i++)
          out << "    ";
        out << std::setw(width) << (std::get<1>(f) * 1000) << " ms : " << std::get<0>(f) << std::endl;
      }
    }
  }
  for (int i = 0; i < depth; i++)
    out << "    ";
  out << std::setw(width) << getTime() * 1000 << " ms : " << name << " (" << nbIterations << " iterations)"
      << std::endl;
}

double Benchmark::closeCSV(const std::string& path, int detailLevel)
{
  std::ofstream out;
  out.open(path);
  double time = closeCSV(out, true, detailLevel);
  out.close();
  return time;
}

double Benchmark::closeCSV(std::ostream& out, bool header, int detailLevel)
{
  if (current == NULL)
    throw std::runtime_error("No active benchmark to close");
  Benchmark* toClose = current;
  // Close the benchmark
  toClose->endSession();
  current = toClose->father;
  // Print header if specified
  if (header)
    printCSVHeader(out);
  // Print benchmark
  toClose->printCSV(out, 0, detailLevel);
  return toClose->getTime();
}

void Benchmark::printCSVHeader(std::ostream& out)
{
  out << "depth,name,father,time" << std::endl;
}

void Benchmark::printCSV(std::ostream& out, int depth, int maxDepth)
{
  // Getting father name
  std::string fatherName("unknown");
  if (father != NULL)
    fatherName = father->name;
  // Printing current informations
  out << depth << "," << name << "," << fatherName << "," << getTime() << std::endl;

  // Print childrens if allowed and found
  if (children.size() > 0 && (maxDepth < 0 || depth < maxDepth))
  {
    for (auto& c : children)
    {
      c.second->printCSV(out, depth + 1, maxDepth);
    }
    // Print the unknown part
    double unknownTime = getTime() - getSubTime();
    out << (depth + 1) << ","
        << "unknown"
        << "," << name << "," << unknownTime << std::endl;
  }
}
}  // namespace rhoban_utils
