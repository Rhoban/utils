#pragma once

#include <cmath>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <iomanip>
#include <iostream>
#include <ostream>
#include <fstream>
#include <Eigen/Geometry>
#include "rhoban_utils/util.h"
#include "rhoban_utils/logging/csv.h"

namespace rhoban_utils
{
// Average two given frames
// weight should be between 0 (frameA) and 1 (frameB)
Eigen::Affine3d averageFrames(Eigen::Affine3d frameA, Eigen::Affine3d frameB, double AtoB);

enum InterpolatePolicy
{
  /**
   * Interpolate linearly between the different values of the history
   * - if t <= begin: return first value
   * - if t >= end: return last value
   * - if begin < t < end:
   *   - Identify the two closest entries (t_0, v_0) and (t_a,v_a)
   *   - Uses linear interpolation between the two of them
   */
  Linear = 0,
  /**
   * Uses last accessible value
   * - if t < begin: return fallback
   */
  Last = 1
};

class HistoryBase
{
public:
  virtual ~HistoryBase()
  {
  }
  virtual void setWindowSize(double window) = 0;
  virtual size_t size() const = 0;
  virtual void startNamedLog(const std::string& sessionName) = 0;
  virtual void freezeNamedLog(const std::string& sessionName) = 0;
  virtual void closeFrozenLog(const std::string& sessionName, std::ostream& os) = 0;
  virtual void loadReplay(std::istream& is, double timeShift = 0.0) = 0;
  virtual double frontTimestamp() const = 0;
  virtual double backTimestamp() const = 0;
  virtual void clear() = 0;
  virtual std::map<std::string, double> requestValue(double time_stamp) const = 0;
  virtual HistoryBase* clone() = 0;
};

/**
 * History
 *
 * Class for storing values and interpolate them back in the past.
 */
template <typename T>
class History : public HistoryBase
{
public:
  typedef std::pair<double, T> TimedValue;

  /**
   * Initialization in timestamp duration
   */
  History(double window, InterpolatePolicy policy = InterpolatePolicy::Linear)
    : _mutex(), _windowSize(window), _values(), _policy(policy)
  {
  }

  /**
   * This is used to abstract writing/reading data from and to a log stream
   */
  virtual TimedValue readValueFromStream(std::istream& is) = 0;
  virtual void writeValueToStream(const TimedValue& value, std::ostream& os) = 0;

  /**
   * Sets the history window size
   */
  void setWindowSize(double window)
  {
    _windowSize = window;
  }

  /**
   * Return the number of internal stored data
   */
  size_t size() const
  {
    return _values.size();
  }

  /**
   * Return first and last recorded point
   */
  TimedValue front() const
  {
    if (_values.size() == 0)
    {
      TimedValue value;
      value.first = 0;
      value.second = fallback();
      return value;
    }
    else
    {
      return *(_values.begin());
    }
  }

  virtual double frontTimestamp() const
  {
    return front().first;
  }

  TimedValue back() const
  {
    if (_values.size() == 0)
    {
      TimedValue value;
      value.first = 0;
      value.second = fallback();
      return value;
    }
    else
    {
      return *(_values.rbegin());
    }
  }

  virtual double backTimestamp() const
  {
    return back().first;
  }

  /**
   * Insert a new value in the container with given timestamp and value
   * if ordered is true, a logic_error is thrown if timestamp is smaller than the largest timestamp contained in history
   */
  void pushValue(double timestamp, T value, bool ordered = true)
  {
    // Lock
    _mutex.lock();
    // Check that timestamp is increasing
    if (ordered && _values.size() > 0 && timestamp < backTimestamp())
    {
      _mutex.unlock();

      std::ostringstream os;
      os << "History invalid timestamp (" << timestamp << " / " << back().first << ")" << std::endl;
      throw std::logic_error(os.str());
    }
    if (_values.count(timestamp) > 0)
    {
      _mutex.unlock();
      return;
    }

    // Insert the value
    _values[timestamp] = value;
    // Shrink the queue if not in logging mode
    if (_windowSize > 0.0)
    {
      _values.erase(_values.begin(), _values.lower_bound(backTimestamp() - _windowSize));
    }

    // Write new entries for all named sessions
    for (auto& namedLog : _activeLogs)
    {
      namedLog.second->operator[](timestamp) = value;
    }
    // Unlock
    _mutex.unlock();
  }

  /**
   * Return either the nearest value or the
   * linear interpolated value associated with
   * given timestamp
   */
  virtual T interpolate(double timestamp) const
  {
    // Lock
    _mutex.lock();

    // Degenerate fallback cases
    if (_values.size() == 0)
    {
      _mutex.unlock();
      return fallback();
    }
    else if (timestamp <= frontTimestamp())
    {
      _mutex.unlock();
      if (_policy == InterpolatePolicy::Last)
        return fallback();
      return front().second;
    }
    else if (timestamp >= backTimestamp())
    {
      _mutex.unlock();
      return back().second;
    }

    // It is now sure that timestamp is in the interval
    auto iterator = _values.upper_bound(timestamp);
    double tsUp = iterator->first;
    T valUp = iterator->second;
    // There is necessary a predecessor since iterator->first > timestamp and _values.begin()->first < timestamp
    iterator--;
    double tsLow = iterator->first;
    T valLow = iterator->second;

    // Unlock
    _mutex.unlock();

    if (_policy == InterpolatePolicy::Last)
      return valLow;

    // Weights
    double wLow = (tsUp - timestamp) / (tsUp - tsLow);
    double wUp = (timestamp - tsLow) / (tsUp - tsLow);

    return doInterpolate(valLow, wLow, valUp, wUp);
  }

  /**
   * Do the actual interpolation between valLow and valHigh
   */
  virtual T doInterpolate(const T& valLow, double wLow, const T& valUp, double wUp) const = 0;
  virtual T fallback() const = 0;

  /**
   * Open a log session with the given name, throws a logic_error if a
   * session with the given name is already opened
   */
  void startNamedLog(const std::string& sessionName)
  {
    _mutex.lock();
    if (_activeLogs.count(sessionName) > 0)
    {
      _mutex.unlock();
      throw std::logic_error(DEBUG_INFO + " there is already a session with name '" + sessionName + "'");
    }
    _activeLogs[sessionName] = std::unique_ptr<std::map<double, T>>(new std::map<double, T>());
    _mutex.unlock();
  }

  /**
   * Move the session with the given name from the active set to the
   * frozen set. Low time consumption.
   */
  void freezeNamedLog(const std::string& sessionName)
  {
    _mutex.lock();
    if (_activeLogs.count(sessionName) == 0)
    {
      _mutex.unlock();
      throw std::logic_error(DEBUG_INFO + " there is no active session with name '" + sessionName + "'");
    }
    if (_frozenLogs.count(sessionName) > 0)
    {
      _mutex.unlock();
      throw std::logic_error(DEBUG_INFO + " there is already a frozen session with name '" + sessionName + "'");
    }
    _frozenLogs[sessionName] = std::move(_activeLogs[sessionName]);
    _activeLogs.erase(sessionName);
    _mutex.unlock();
  }

  /**
   * Close a log session with the given name, throws a logic_error if
   * there is no frozen session with the given name opened. High time
   * consumption.
   */
  void closeFrozenLog(const std::string& sessionName, std::ostream& os)
  {
    _mutex.lock();
    if (_frozenLogs.count(sessionName) == 0)
    {
      _mutex.unlock();
      throw std::logic_error(DEBUG_INFO + " there is no open session with name '" + sessionName + "'");
    }
    std::unique_ptr<std::map<double, T>> values = std::move(_frozenLogs[sessionName]);
    _frozenLogs.erase(sessionName);
    _mutex.unlock();

    writeBinary(*values, os);
  }

  /**
   * Read data from given input stream
   * until either the stream end or the first
   * commented "#" line.
   * If binary is true, log file is read in
   * binary format
   * Optional time shift is apply on read timestamp
   */
  void loadReplay(std::istream& is, double timeShift = 0.0)
  {
    _mutex.lock();
    // Clean the container
    _values.clear();
    // Read the number of data
    size_t size = 0;

    is.read((char*)&size, sizeof(size_t));

    // Read the input stream
    while (true)
    {
      // Extract one data point
      TimedValue newValue;

      if (size == 0)
      {
        _mutex.unlock();
        return;
      }

      newValue = readValueFromStream(is);
      size--;

      // Apply time shift
      newValue.first += timeShift;

      // Check that timestamp is increasing
      if (_values.size() > 0 && newValue.first <= frontTimestamp())
      {
        _mutex.unlock();
        throw std::runtime_error("History invalid timestamp");
      }
      // Insert the value
      _values[newValue.first] = newValue.second;
    }
  }

  /**
   * Getting all values
   */
  std::map<double, T> getValues()
  {
    return _values;
  }

  /**
   * Clearing history values
   */
  void clear()
  {
    _values.clear();
  }

protected:
  /**
   * Write the history to the provided stream
   */
  void writeBinary(const std::map<double, T>& values, std::ostream& os)
  {
    // When writing in binary, use all values
    size_t size = values.size();
    os.write((const char*)&size, sizeof(size_t));
    for (const auto& it : values)
    {
      // Write binary data
      writeValueToStream(it, os);
    }
  }

  /**
   * Mutex for concurent access
   */
  mutable std::mutex _mutex;

  /**
   * Rolling buffer size in timestamp
   */
  double _windowSize;

  /**
   * Values container indexed
   * by their timestamp
   */
  std::map<double, T> _values;

  /**
   * Named log sessions to which the object is actively writting
   */
  std::map<std::string, std::unique_ptr<std::map<double, T>>> _activeLogs;
  /**
   * Named log sessions waiting to be written
   */
  std::map<std::string, std::unique_ptr<std::map<double, T>>> _frozenLogs;

  /**
   * How are values being interpolated
   */
  InterpolatePolicy _policy;
};

class HistoryDouble : public History<double>
{
public:
  HistoryDouble(double window = 2.0, InterpolatePolicy policy = InterpolatePolicy::Linear);

  double doInterpolate(const double& valLow, double wLow, const double& valUp, double wUp) const;
  double fallback() const;

  TimedValue readValueFromStream(std::istream& is);
  void writeValueToStream(const TimedValue& value, std::ostream& os);

  std::map<std::string, double> requestValue(double time_stamp) const;
  HistoryBase* clone();
};

class HistoryBool : public History<bool>
{
public:
  HistoryBool(double window = 2.0, InterpolatePolicy policy = InterpolatePolicy::Linear);

  bool doInterpolate(const bool& valLow, double wLow, const bool& valUp, double wUp) const;
  bool fallback() const;

  TimedValue readValueFromStream(std::istream& is);
  void writeValueToStream(const TimedValue& value, std::ostream& os);

  std::map<std::string, double> requestValue(double time_stamp) const;
  HistoryBase* clone();
};

class HistoryAngle : public HistoryDouble
{
public:
  HistoryAngle(double window = 2.0, InterpolatePolicy policy = InterpolatePolicy::Linear);

  double doInterpolate(const double& valLow, double wLow, const double& valHigh, double wHigh) const;
  HistoryBase* clone();
};

class HistoryPose : public History<Eigen::Affine3d>
{
public:
  HistoryPose(double window = 2.0, InterpolatePolicy policy = InterpolatePolicy::Linear);

  Eigen::Affine3d doInterpolate(const Eigen::Affine3d& valLow, double wLow, const Eigen::Affine3d& valHigh,
                                double wHigh) const;
  Eigen::Affine3d fallback() const;

  /**
   * Return the transformation from pose at t1 to pose at t2
   */
  Eigen::Affine3d getDiff(double t1, double t2) const;

  TimedValue readValueFromStream(std::istream& is);
  void writeValueToStream(const TimedValue& value, std::ostream& os);

  std::map<std::string, double> requestValue(double time_stamp) const;
  HistoryBase* clone();
};

class HistoryVector3d : public History<Eigen::Vector3d>
{
public:
  HistoryVector3d(double window = 2.0, InterpolatePolicy policy = InterpolatePolicy::Linear);

  Eigen::Vector3d doInterpolate(const Eigen::Vector3d& valLow, double wLow, const Eigen::Vector3d& valHigh,
                                double wHigh) const;
  Eigen::Vector3d fallback() const;

  TimedValue readValueFromStream(std::istream& is);
  void writeValueToStream(const TimedValue& value, std::ostream& os);

  std::map<std::string, double> requestValue(double time_stamp) const;
  HistoryBase* clone();
};

class HistoryCollection
{
public:
  HistoryCollection(double window = 2.0);
  // Copy constructor (might not be thread safe if other object is accessed in same time)
  HistoryCollection(const HistoryCollection& other);

  virtual ~HistoryCollection();

  template <typename T>
  T* get(std::string name)
  {
    if (!_histories.count(name))
    {
      T* h = new T(window);
      _histories[name] = h;
    }

    T* h = dynamic_cast<T*>(_histories[name]);
    if (h == nullptr)
    {
      std::ostringstream os;
      os << "Asking for history of bad type (" << name << ")";
      throw std::logic_error(os.str());
    }

    return h;
  }

  // Types for binary log file header
  static const int HISTORY_NUMBER = 0;
  static const int HISTORY_ANGLE = 1;
  static const int HISTORY_BOOLEAN = 2;
  static const int HISTORY_POSE = 3;

  HistoryDouble* number(std::string name)
  {
    return get<HistoryDouble>(name);
  }

  HistoryAngle* angle(std::string name)
  {
    return get<HistoryAngle>(name);
  }

  HistoryPose* pose(std::string name)
  {
    return get<HistoryPose>(name);
  }

  HistoryBool* boolean(std::string name)
  {
    return get<HistoryBool>(name);
  }

  /**
   * Export the history collection to the csv file `filename`.
   * The values are taken at: min, min+dt, min+2*dt, ..., max
   * where min and max are rexpectively the front and back timestamps.
   * dt is in s.
   */
  void exportToCSV(double dt, std::string filename, char separator = ',') const;

  /**
   * Start and stop (save) a named log session
   */
  void startNamedLog(const std::string& filePath);
  void stopNamedLog(const std::string& filePath);

  /**
   *  Load the replays from a given file stream
   */
  void loadReplays(const std::string& filePath, bool oldFormat = false);

  /**
   * Find the smallest and the biggest timestamp
   */
  double smallestTimestamp() const;
  double biggestTimestamp() const;

  void clear();

  std::map<std::string, HistoryBase*>& entries();

  std::map<std::string, double> requestValues(double time_stamp) const;

protected:
  double window;
  std::mutex mutex;
  std::map<std::string, HistoryBase*> _histories;
};

}  // namespace rhoban_utils
