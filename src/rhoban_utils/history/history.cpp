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

uint8_t HistoryDouble::type()
{
  return 1;
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

HistoryAngle::HistoryAngle(double window) : HistoryDouble(window)
{
}

uint8_t HistoryAngle::type()
{
  return 2;
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

uint8_t HistoryPose::type()
{
  return 3;
}

HistoryPose::TimedValue HistoryPose::readValueFromStream(std::istream& is)
{
  HistoryPose::TimedValue value;

  // XXX: Todo

  return value;
}

void HistoryPose::writeValueToStream(const HistoryPose::TimedValue& value, std::ostream& os)
{
  // XXX: Todo
}

Eigen::Affine3d HistoryPose::doInterpolate(Eigen::Affine3d valLow, double wLow, Eigen::Affine3d valHigh, double wHigh) const
{
  // Slerp for orientation
  Eigen::Quaterniond qLow(valLow.rotation());
  Eigen::Quaterniond qHigh(valHigh.rotation());
  Eigen::Quaterniond q = qLow.slerp(wHigh, qHigh);

  // Weighted average for translation
  Eigen::Vector3d tLow(valLow.translation().x(), valLow.translation().y(), valLow.translation().z());
  Eigen::Vector3d tHigh(valHigh.translation().x(), valHigh.translation().y(), valHigh.translation().z());
  Eigen::Vector3d t = wLow*tLow + wHigh*tHigh;

  Eigen::Affine3d result;
  result.fromPositionOrientationScale(t, q, Eigen::Vector3d(1, 1, 1));

  return result;
}

Eigen::Affine3d HistoryPose::fallback() const
{
  return Eigen::Affine3d::Identity();
}

HistoryCollection::HistoryCollection() : mutex()
{
}

HistoryCollection::~HistoryCollection()
{
  clear();
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
    uint8_t type;
    file.read((char*)&length, sizeof(size_t));
    file.read((char*)&type, sizeof(type));
    file.read(buffer, length);
    buffer[length] = '\0';
    std::string name(buffer);

    // Retrieve all data for current key
    if (type == 1)
    {
      _histories[name] = new HistoryDouble();
    }
    else if (type == 2)
    {
      _histories[name] = new HistoryAngle();
    }
    else if (type == 3)
    {
      _histories[name] = new HistoryPose();
    }
    else
    {
      throw std::runtime_error("Unable to load history: bad type");
    }

    _histories[name]->loadReplay(file, 0.0);
    std::cout << "Loading " << name << " with " << _histories[name]->size() << " points" << std::endl;
  }
  // Start replay
  // XXX ???
  // _replayTimestamp = _histories["read:head_pitch"].front().first;

  // Close read file
  file.close();
  mutex.unlock();
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
    try {
    it.second->freezeNamedLog(filePath);
    } catch (std::logic_error error) {
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
    uint8_t type = it.second->type();
    file.write((const char*)(&type), sizeof(type));
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
    delete entry.second;
  }

  _histories.clear();
  mutex.unlock();
}

}  // namespace rhoban_utils
