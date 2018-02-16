#pragma once

#include <json/json.h>

#include <Eigen/Core>

#include <exception>

namespace rhoban_utils
{

class JsonParsingError : public std::runtime_error
{
public:
  JsonParsingError(const std::string & what_arg);
};

Json::Value file2Json(const std::string & path);

class JsonSerializable
{
public:
  JsonSerializable();
  virtual ~JsonSerializable();

  virtual std::string getClassName() const = 0;
  
  /// loads the object from the default file
  /// the default filename is the name of the class + .json
  virtual void loadFile();

  /// Load the file at the given path
  void loadFile(const std::string & file_path);

  /// Separation between json_file and directory is used to ensure
  /// possibility to use relative_path (from dir_path)
  /// dir_path should end by a "/"
  void loadFile(const std::string & json_file,
                const std::string & dir_path);

  /// Serializes and saves to a file using default filename
  void saveFile() const;

  /// Serializes and saves to a file using given filename
  /// if factory_style is true, uses toFactoryJson (allows choosing type during loading)
  void saveFile(const std::string &filename, bool factory_style = false) const;

  /// Deserializes from a json content found in 'dir_name'
  virtual void fromJson(const Json::Value & json_value,
                        const std::string & dir_name) = 0;

  /// Represent current object as a Json::Value
  virtual Json::Value toJson() const = 0;

  /// Represent current object as a Json::Value containing information on class for factories
  Json::Value toFactoryJson() const;

  
  /// Read the content of the object if v[key] exists
  /// Otherwise: throws a JsonParsingError
  void read(const Json::Value & v, const std::string & key, const std::string & dir_name = "./");
  /// Read the content of the object if v[key] exists
  /// Do not throw an exception if v[key] is not defined
  /// Still throws an exception if v[key] exists with an invalid content
  void tryRead(const Json::Value & v, const std::string & key, const std::string & dir_name = "./");

};

/// Throws a JsonParsingError if v[key] does not exist
void checkMember(const Json::Value & v, const std::string & key);

/// There is no default getJsonVal
template <typename T> T getJsonVal(const Json::Value & v) = delete;

template <> bool        getJsonVal<bool>       (const Json::Value & v);
template <> int         getJsonVal<int>        (const Json::Value & v);
template <> float       getJsonVal<float>      (const Json::Value & v);
template <> double      getJsonVal<double>     (const Json::Value & v);
template <> std::string getJsonVal<std::string>(const Json::Value & v);

/// There is no default val2Json
template <typename T> Json::Value val2Json(const T & val) = delete;

template <> Json::Value val2Json<bool>(const bool & val);
template <> Json::Value val2Json<int>(const int & val);
template <> Json::Value val2Json<float>(const float & val);
template <> Json::Value val2Json<double>(const double & val);
template <> Json::Value val2Json<std::string>(const std::string & val);


/// Return an object of type 'T' if v[key] exists and is of type 'T'.
/// Otherwise: throws a JsonParsingError
template <typename T> T read(const Json::Value & v, const std::string & key)
{
  checkMember(v,key);
  try {
    return getJsonVal<T>(v[key]);
  } catch (const JsonParsingError & error) {
    throw JsonParsingError(error.what() + std::string(" at '") + key + "'");   
  }
}
  
/// - if v[key] exists and has type 'T', write the value in ptr
/// - if v[key] does not exist, do not modify ptr
/// - if v[key] exists but has inappropriate type, throws a JsonParsingError
template <typename T>
void tryRead(const Json::Value & v, const std::string & key, T * ptr)
{
  if (v.isObject() && v.isMember(key)) {
    *ptr = read<T>(v, key);
  }
}

template <typename T>
std::vector<T> readVector(const Json::Value & v, const std::string & key)
{
  checkMember(v,key);
  if (!v[key].isArray()) {
    throw JsonParsingError("readVector(): Value at '" + key + "' is not an array");
  }
  std::vector<T> result;
  for (Json::ArrayIndex idx=0; idx < v[key].size();idx++) {
    result.push_back(getJsonVal<T>(v[key][idx]));
  }
  return result;
}

template <typename T>
void tryReadVector(const Json::Value & v, const std::string & key, std::vector<T> * ptr)
{
  if (v.isObject() && v.isMember(key)) {
    *ptr = readVector<T>(v, key);
  }
}

template <typename T>
static Json::Value vector2Json(const std::vector<T> & values)
{
  Json::Value v;
  for (Json::ArrayIndex idx = 0; idx < values.size(); idx++) {
    v[idx] = val2Json(values[idx]);
  }
  return v;
}

Json::Value vector2Json(const Eigen::VectorXd & v);
Json::Value matrix2Json(const Eigen::MatrixXd & m);

//TODO: a solution has to be found for fixed size vectors!
template <> Eigen::Vector2d getJsonVal<Eigen::Vector2d>(const Json::Value & v);
template <> Eigen::Vector3d getJsonVal<Eigen::Vector3d>(const Json::Value & v);
template <> Eigen::VectorXd getJsonVal<Eigen::VectorXd>(const Json::Value & v);
template <> Eigen::MatrixXd getJsonVal<Eigen::MatrixXd>(const Json::Value & v);

template <typename T>
std::vector<T> readVector(const Json::Value & v,
                          const std::string & dir_name,
                          std::function<T(const Json::Value & v,
                                          const std::string & dir_name)> builder)
{
  if (!v.isArray()) {
    throw JsonParsingError("readVector:: Expecting an array");
  }
  std::vector<T> result;
  for (Json::ArrayIndex idx = 0; idx < v.size(); idx++){
    result.push_back(builder(v[idx],dir_name));
  }
  return result;
}

template <typename T>
std::vector<T> readVector(const Json::Value & v,
                          const std::string & vec_name,
                          const std::string & dir_name,
                          std::function<T(const Json::Value & v, const std::string & dir_name)> builder)
{
  checkMember(v,vec_name);
  return readVector(v[vec_name], dir_name, builder);
}

template <typename T>
void tryReadVector(const Json::Value & v,
                   const std::string & vec_name,
                   const std::string & dir_name,
                   std::function<T(const Json::Value & v,
                                   const std::string & dir_name)> builder,
                   std::vector<T> * ptr)
{
  if (v.isObject() && v.isMember(vec_name)) {
    *ptr = readVector(v, vec_name, dir_name, builder);
  }
}

template <typename T>
Json::Value map2Json(const std::map<std::string,T> & values)
{
  Json::Value v;
  for (const auto & pair : values) {
    v[pair.first] = val2Json(pair.second);
  }
  return v;
}


template <typename T>
std::map<std::string, T> readMap(const Json::Value & v, const std::string & map_key) {
  checkMember(v,map_key);
  if (!v[map_key].isObject()) {
    throw JsonParsingError("readMap(): Value at '" + map_key + "' is not an object");
  }
  // Parse entries:
  const Json::Value & map_v = v[map_key];
  std::map<std::string, T> result;
  for (Json::ValueConstIterator it = map_v.begin(); it != map_v.end(); it++) {
    const std::string & key = it.name();
    result[key] = getJsonVal<T>(map_v[key]);
  }
  return result;
}


template <typename T>
void tryReadMap(const Json::Value & v, const std::string & key, std::map<std::string, T> * ptr)
{
  if (v.isObject() && v.isMember(key)) {
    *ptr = readMap<T>(v, key);
  }
}


/// Read a map which has the following structure using the given builder
/// {
///   "key" : "value",
///   ...
/// }
template <typename T>
std::map<std::string,T> readMap(const Json::Value & v,
                                const std::string & dir_name,
                                std::function<T(const Json::Value & v, const std::string & dir_name)> builder)
{
  if (!v.isObject()) {
    throw JsonParsingError("readMap:: Expecting an object");
  }
  // Parse entries:
  std::map<std::string, T> result;
  for (Json::ValueConstIterator it = v.begin(); it != v.end(); it++) {
    const std::string & key = it.name();
    result[key] = builder(v[key], dir_name);
  }
  return result;
}

template <typename T>
std::map<std::string,T> readMap(const Json::Value & v,
                                const std::string & map_name,
                                const std::string & dir_name,
                                std::function<T(const Json::Value & v, const std::string & dir_name)> builder)
{
  checkMember(v,map_name);
  return readMap(v[map_name], dir_name, builder);
}

}
