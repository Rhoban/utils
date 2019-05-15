#include <rhoban_utils/angle.h>
#include "rhoban_utils/history/history.h"
#include "rhoban_utils/util.h"

namespace rhoban_utils
{
HistoryDouble::HistoryDouble(double window) : History(window)
{
}

double HistoryDouble::doInterpolate(double valLow, double wLow, double valUp, double wUp) const
{
  return wLow * valLow + wUp * valUp;
}

double HistoryDouble::fallback() const
{
  return 0.0;
}

HistoryDouble::TimedValue HistoryDouble::readValueFromStream(std::istream& is)
{
  TimedValue value;

  is.read((char*)&value.first, sizeof(double));
  is.read((char*)&value.second, sizeof(double));

  return value;
}

void HistoryDouble::writeValueToStream(const TimedValue& value, std::ostream& os)
{
  os.write((const char*)&(value.first), sizeof(double));
  os.write((const char*)&(value.second), sizeof(double));
}

std::map<std::string, double> HistoryDouble::requestValue(double time_stamp) const
{
  std::map<std::string, double> res{ { "value", HistoryDouble::interpolate(time_stamp) } };
  return res;
}

HistoryAngle::HistoryAngle(double window) : HistoryDouble(window)
{
}

double HistoryAngle::doInterpolate(double valLow, double wLow, double valUp, double wUp) const
{
  Angle angleLow(rad2deg(valLow));
  Angle angleUp(rad2deg(valUp));
  auto result = Angle::weightedAverage(angleLow, wLow, angleUp, wUp);

  return deg2rad(result.getSignedValue());
}

HistoryPose::HistoryPose(double window) : History(window)
{
}

HistoryPose::TimedValue HistoryPose::readValueFromStream(std::istream& is)
{
  HistoryPose::TimedValue value;

  double values[7];

  is.read((char*)&value.first, sizeof(double));
  is.read((char*)&values, sizeof(values));

  value.second.fromPositionOrientationScale(Eigen::Vector3d(values[0], values[1], values[2]),
                                            Eigen::Quaterniond(values[3], values[4], values[5], values[6]),
                                            Eigen::Vector3d(1, 1, 1));

  return value;
}

void HistoryPose::writeValueToStream(const HistoryPose::TimedValue& value, std::ostream& os)
{
  auto translation = value.second.translation();
  Eigen::Quaterniond q(value.second.rotation());

  double values[7] = { translation.x(), translation.y(), translation.z(), q.w(), q.x(), q.y(), q.z() };

  os.write((const char*)&(value.first), sizeof(double));
  os.write((const char*)&(values), sizeof(values));
}

Eigen::Affine3d HistoryPose::doInterpolate(Eigen::Affine3d valLow, double wLow, Eigen::Affine3d valHigh,
                                           double wHigh) const
{
  // Slerp for orientation
  Eigen::Quaterniond qLow(valLow.rotation());
  Eigen::Quaterniond qHigh(valHigh.rotation());
  Eigen::Quaterniond q = qLow.slerp(wHigh, qHigh);

  // Weighted average for translation
  Eigen::Vector3d tLow(valLow.translation().x(), valLow.translation().y(), valLow.translation().z());
  Eigen::Vector3d tHigh(valHigh.translation().x(), valHigh.translation().y(), valHigh.translation().z());
  Eigen::Vector3d t = wLow * tLow + wHigh * tHigh;

  Eigen::Affine3d result;
  result.fromPositionOrientationScale(t, q, Eigen::Vector3d(1, 1, 1));

  return result;
}

Eigen::Affine3d HistoryPose::fallback() const
{
  return Eigen::Affine3d::Identity();
}

std::map<std::string, double> HistoryPose::requestValue(double time_stamp) const
{
  std::map<std::string, double> res;
  Eigen::Affine3d pose = HistoryPose::interpolate(time_stamp);
  res["tx"] = pose.translation().x();
  res["ty"] = pose.translation().y();
  res["tz"] = pose.translation().z();

  Eigen::Quaterniond quat(pose.rotation());
  res["qx"] = quat.x();
  res["qy"] = quat.y();
  res["qz"] = quat.z();
  res["qw"] = quat.w();
  return res;
}

HistoryCollection::HistoryCollection(double window) : mutex(), window(window)
{
}

HistoryCollection::~HistoryCollection()
{
  for (auto& entry : _histories)
  {
    delete entry.second;
  }
}

void HistoryCollection::loadReplays(const std::string& filePath)
{
  clear();

  mutex.lock();

  std::ifstream file(filePath.c_str());
  // Check file
  if (!file.is_open())
  {
    mutex.unlock();
    throw std::runtime_error("HistoryCollection unable to read file: '" + filePath + "'");
  }

  // Binary format
  while (true)
  {
    if (!file.good() || file.peek() == EOF)
    {
      break;
    }
    size_t length = 0;
    char buffer[256];
    file.read((char*)&length, sizeof(size_t));
    file.read(buffer, length);
    buffer[length] = '\0';
    std::string name(buffer);

    // Retrieve all data for current key
    if (!_histories.count(name))
    {
      std::ostringstream os;
      os << "Unable to load history, unknown name \"" << name << "\"";
      throw std::runtime_error(os.str());
    }

    _histories[name]->loadReplay(file, 0.0);
    std::cout << "Loading " << name << " with " << _histories[name]->size() << " points" << std::endl;
  }

  // Close read file
  file.close();
  mutex.unlock();
}

double HistoryCollection::smallestTimestamp() const
{
  bool has = false;
  double smallestTimestamp = -1;

  for (auto& entry : _histories)
  {
    if (entry.second->size() > 0 && (!has || entry.second->frontTimestamp() < smallestTimestamp))
    {
      has = true;
      smallestTimestamp = entry.second->frontTimestamp();
    }
  }

  return smallestTimestamp;
}

double HistoryCollection::biggestTimestamp() const
{
  bool has = false;
  double biggestTimestamp = -1;

  for (auto& entry : _histories)
  {
    if (entry.second->size() > 0 && (!has || entry.second->backTimestamp() > biggestTimestamp))
    {
      has = true;
      biggestTimestamp = entry.second->backTimestamp();
    }
  }

  return biggestTimestamp;
}

void HistoryCollection::startNamedLog(const std::string& filePath)
{
  mutex.lock();
  for (auto& it : _histories)
  {
    it.second->startNamedLog(filePath);
  }
  mutex.unlock();
}

void HistoryCollection::stopNamedLog(const std::string& filePath)
{
  mutex.lock();

  /// First, freeze all histories for the session name
  for (auto& it : _histories)
  {
    try
    {
      it.second->freezeNamedLog(filePath);
    }
    catch (std::logic_error error)
    {
      mutex.unlock();
      throw error;
    }
  }
  // Open log file
  std::ofstream file(filePath.c_str());
  // Check file
  if (!file.is_open())
  {
    mutex.unlock();
    throw std::runtime_error(DEBUG_INFO + "unable to write to file '" + filePath + "'");
  }
  /// Second write logs
  for (auto& it : _histories)
  {
    size_t length = it.first.length();
    file.write((const char*)(&length), sizeof(size_t));
    file.write((const char*)(it.first.c_str()), length);
    it.second->closeFrozenLog(filePath, file);
  }

  mutex.unlock();
}

void HistoryCollection::clear()
{
  mutex.lock();
  for (auto& entry : _histories)
  {
    entry.second->clear();
  }
  mutex.unlock();
}

std::map<std::string, HistoryBase*>& HistoryCollection::entries()
{
  return _histories;
}

std::map<std::string, double> HistoryCollection::requestValues(double time_stamp) const
{
  std::map<std::string, double> res;
  for (auto& entry : _histories)
  {
    std::map<std::string, double> tmp = entry.second->requestValue(time_stamp);
    for (auto& el : tmp)
    {
      res[entry.first + ":" + el.first] = el.second;
    }
  }
  return res;
}

// dt [s]
void HistoryCollection::exportToCSV(double dt, std::string filename, char separator) const
{
  CSV* csv = new rhoban_utils::CSV();
  csv->open(filename, separator);

  std::map<std::string, double> values;

  double tmp_t = HistoryCollection::smallestTimestamp();
  double max_t = HistoryCollection::biggestTimestamp();
  do
  {
    csv->push("time", tmp_t);
    values = HistoryCollection::requestValues(tmp_t);
    for (auto& val : values)
      csv->push(val.first, val.second);
    csv->newLine();
    tmp_t += dt;
  } while (tmp_t < max_t);

  csv->close();
}

}  // namespace rhoban_utils
