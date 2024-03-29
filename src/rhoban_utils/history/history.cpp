#include <rhoban_utils/angle.h>
#include "rhoban_utils/history/history.h"
#include "rhoban_utils/util.h"

namespace rhoban_utils
{
Eigen::Affine3d averageFrames(Eigen::Affine3d frameA, Eigen::Affine3d frameB, double AtoB)
{
  // Slerp for orientation
  Eigen::Quaterniond quaternionA(frameA.rotation());
  Eigen::Quaterniond quaternionB(frameB.rotation());
  Eigen::Quaterniond q = quaternionA.slerp(AtoB, quaternionB);

  // Weighted average for translation
  Eigen::Vector3d translationA(frameA.translation().x(), frameA.translation().y(), frameA.translation().z());
  Eigen::Vector3d translationB(frameB.translation().x(), frameB.translation().y(), frameB.translation().z());
  Eigen::Vector3d t = translationA * (1 - AtoB) + translationB * AtoB;

  Eigen::Affine3d result;
  result.fromPositionOrientationScale(t, q, Eigen::Vector3d(1, 1, 1));

  return result;
}

HistoryDouble::HistoryDouble(double window, InterpolatePolicy policy) : History(window, policy)
{
}

double HistoryDouble::doInterpolate(const double& valLow, double wLow, const double& valUp, double wUp) const
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

HistoryBase* HistoryDouble::clone()
{
  HistoryBase* copy = new HistoryDouble(_windowSize);
  *copy = *this;
  return copy;
}

HistoryBase* HistoryAngle::clone()
{
  HistoryBase* copy = new HistoryAngle(_windowSize);
  *copy = *this;
  return copy;
}

std::map<std::string, double> HistoryDouble::requestValue(double time_stamp) const
{
  std::map<std::string, double> res{ { "value", HistoryDouble::interpolate(time_stamp) } };
  return res;
}

HistoryBool::HistoryBool(double window, InterpolatePolicy policy) : History(window, policy)
{
}

bool HistoryBool::doInterpolate(const bool& valLow, double wLow, const bool& valUp, double wUp) const
{
  (void)wUp;
  if (wLow > 0.5)
  {
    return valLow;
  }
  else
  {
    return valUp;
  }
}

bool HistoryBool::fallback() const
{
  return false;
}

HistoryBool::TimedValue HistoryBool::readValueFromStream(std::istream& is)
{
  TimedValue value;

  is.read((char*)&value.first, sizeof(double));
  is.read((char*)&value.second, sizeof(bool));

  return value;
}

void HistoryBool::writeValueToStream(const TimedValue& value, std::ostream& os)
{
  os.write((const char*)&(value.first), sizeof(double));
  os.write((const char*)&(value.second), sizeof(bool));
}

HistoryBase* HistoryBool::clone()
{
  HistoryBase* copy = new HistoryBool(_windowSize);
  *copy = *this;
  return copy;
}

std::map<std::string, double> HistoryBool::requestValue(double time_stamp) const
{
  std::map<std::string, double> res{ { "value", HistoryBool::interpolate(time_stamp) ? 1.0 : 0.0 } };
  return res;
}

HistoryAngle::HistoryAngle(double window, InterpolatePolicy policy) : HistoryDouble(window, policy)
{
}

double HistoryAngle::doInterpolate(const double& valLow, double wLow, const double& valUp, double wUp) const
{
  Angle angleLow(rad2deg(valLow));
  Angle angleUp(rad2deg(valUp));
  auto result = Angle::weightedAverage(angleLow, wLow, angleUp, wUp);

  return deg2rad(result.getSignedValue());
}

HistoryPose::HistoryPose(double window, InterpolatePolicy policy) : History(window, policy)
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

Eigen::Affine3d HistoryPose::doInterpolate(const Eigen::Affine3d& valLow, double wLow, const Eigen::Affine3d& valHigh,
                                           double wHigh) const
{
  (void)wLow;
  return averageFrames(valLow, valHigh, wHigh);
}

Eigen::Affine3d HistoryPose::fallback() const
{
  return Eigen::Affine3d::Identity();
}

Eigen::Affine3d HistoryPose::getDiff(double t1, double t2) const
{
  // We use x as an arbitrary common basis
  Eigen::Affine3d x_from_t1 = interpolate(t1).inverse();
  Eigen::Affine3d t2_from_x = interpolate(t2);
  return t2_from_x * x_from_t1;
}

HistoryBase* HistoryPose::clone()
{
  HistoryBase* copy = new HistoryPose(_windowSize);
  *copy = *this;
  return copy;
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

HistoryVector3d::HistoryVector3d(double window, InterpolatePolicy policy) : History(window, policy)
{
}

HistoryVector3d::TimedValue HistoryVector3d::readValueFromStream(std::istream& is)
{
  HistoryVector3d::TimedValue value;

  double values[3];

  is.read((char*)&value.first, sizeof(double));
  is.read((char*)&values, sizeof(values));

  value.second = Eigen::Vector3d(values[0], values[1], values[2]);

  return value;
}

void HistoryVector3d::writeValueToStream(const HistoryVector3d::TimedValue& value, std::ostream& os)
{
  double values[3] = { value.second.x(), value.second.y(), value.second.z() };

  os.write((const char*)&(value.first), sizeof(double));
  os.write((const char*)&(values), sizeof(values));
}

Eigen::Vector3d HistoryVector3d::doInterpolate(const Eigen::Vector3d& valLow, double wLow,
                                               const Eigen::Vector3d& valHigh, double wHigh) const
{
  return (valLow * wLow + valHigh * wHigh) / (wLow + wHigh);
}

Eigen::Vector3d HistoryVector3d::fallback() const
{
  return Eigen::Vector3d::Zero();
}

HistoryBase* HistoryVector3d::clone()
{
  HistoryBase* copy = new HistoryVector3d(_windowSize);
  *copy = *this;
  return copy;
}

std::map<std::string, double> HistoryVector3d::requestValue(double time_stamp) const
{
  std::map<std::string, double> res;
  Eigen::Vector3d pos = interpolate(time_stamp);
  res["x"] = pos.x();
  res["y"] = pos.y();
  res["z"] = pos.z();
  return res;
}

HistoryCollection::HistoryCollection(double window) : window(window), mutex()
{
}

HistoryCollection::HistoryCollection(const HistoryCollection& other)
{
  window = other.window;

  for (auto& entry : other._histories)
  {
    _histories[entry.first] = entry.second->clone();
  }
}

HistoryCollection::~HistoryCollection()
{
  for (auto& entry : _histories)
  {
    delete entry.second;
  }
}

void HistoryCollection::loadReplays(const std::string& filePath, bool oldFormat, bool verbose)
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
    char type;
    if (!oldFormat)
    {
      file.read(&type, 1);
    }

    char buffer[256];
    file.read((char*)&length, sizeof(size_t));
    file.read(buffer, length);
    buffer[length] = '\0';
    std::string name(buffer);

    if (oldFormat)
    {
      if (!_histories.count(name))
      {
        throw std::runtime_error("Entry was not declared in (old-style) history: " + type);
      }
    }
    else
    {
      if (type == HISTORY_NUMBER)
      {
        number(name);
      }
      else if (type == HISTORY_ANGLE)
      {
        angle(name);
      }
      else if (type == HISTORY_BOOLEAN)
      {
        boolean(name);
      }
      else if (type == HISTORY_POSE)
      {
        pose(name);
      }
      else
      {
        throw std::runtime_error("Loading replay with unknown type " + type);
      }
    }

    _histories[name]->loadReplay(file, 0.0);
    if (verbose)
    {
      std::cout << "Loading " << name << " with " << _histories[name]->size() << " points" << std::endl;
    }
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

std::vector<double> HistoryCollection::getTimestamps()
{
  std::vector<double> timestamps;
  for (auto& it : _histories)
  {
    for (double ts: it.second->getTimestamps())
    {
      timestamps.push_back(ts);
    }
    if (timestamps.size() == 0)
    {
      continue;
    }
    return timestamps;
  }
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
    char type = 0;

    if (dynamic_cast<HistoryAngle*>(it.second) != nullptr)
    {
      type = HISTORY_ANGLE;
    }
    else if (dynamic_cast<HistoryDouble*>(it.second) != nullptr)
    {
      type = HISTORY_NUMBER;
    }
    else if (dynamic_cast<HistoryBool*>(it.second) != nullptr)
    {
      type = HISTORY_BOOLEAN;
    }
    else if (dynamic_cast<HistoryPose*>(it.second) != nullptr)
    {
      type = HISTORY_POSE;
    }
    else
    {
      throw std::runtime_error("An history in collection (" + it.first +
                               ") has unknown type and can't be saved to log file");
    }

    file.write(&type, 1);
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
  std::cout << "Exporting from " << tmp_t << " to " << max_t << std::endl;
  int i = 0;
  do
  {
    csv->push("time", tmp_t);
    values = HistoryCollection::requestValues(tmp_t);
    for (auto& val : values)
      csv->push(val.first, val.second);
    csv->newLine();

    i++;
    if (i > 1000)
    {
      i = 0;
      std::cout << "com_x at " << tmp_t << " is " << values["read:com_x:value"] << std::endl;
    }

    tmp_t += dt;
  } while (tmp_t < max_t);

  csv->close();
}

}  // namespace rhoban_utils
