#pragma once

#include "rhoban_utils/io_tools.h"
#include "rhoban_utils/util.h"
#include "rhoban_utils/serialization/json_serializable.h"

#include <json/json.h>

#include <fstream>
#include <functional>
#include <memory>
#include <string>
#include <map>

namespace rhoban_utils
{
/// This class implements a factory pattern. It can be used for producing
/// various objects inheriting from class T. More specific initialization can be
/// achieved by using data from a Json::Value
template <class T>
class Factory
{
public:
  Factory()
  {
  }

  /// Builder which takes no argument
  typedef std::function<std::unique_ptr<T>()> Builder;
  /// JsonBuilder can use json data
  typedef std::function<std::unique_ptr<T>(const Json::Value& json_val, const std::string& dir)> JsonBuilder;
  /// Builder using an input stream to customize the created object @see StreamSerializable
  typedef std::function<std::unique_ptr<T>(std::istream& in, int* nb_bytes_read)> StreamBuilder;

  JsonBuilder getBuilder(const std::string& class_name) const
  {
    try
    {
      return json_builders.at(class_name);
    }
    catch (const std::out_of_range& exc)
    {
      std::ostringstream oss;
      oss << "Factory: type '" << class_name << "' is not registered" << std::endl;
      listBuilders(oss);
      throw JsonParsingError(oss.str());
    }
  }

  Builder getBuilder(int id) const
  {
    try
    {
      return builders_by_id.at(id);
    }
    catch (const std::out_of_range& exc)
    {
      std::ostringstream oss;
      oss << "Factory: id '" << id << "' is not registered";
      throw std::out_of_range(oss.str());
    }
  }

  StreamBuilder getStreamBuilder(int id) const
  {
    try
    {
      return stream_builders_by_id.at(id);
    }
    catch (const std::out_of_range& exc)
    {
      std::ostringstream oss;
      oss << "Factory: id '" << id << "' is not registered";
      throw std::out_of_range(oss.str());
    }
  }

  std::unique_ptr<T> build(const std::string& class_name) const
  {
    Json::Value v(Json::ValueType::objectValue);
    v["class name"] = class_name;
    v["content"] = Json::Value();
    return build(v);
  }

  /// Reducing ambiguity
  std::unique_ptr<T> build(const char* class_name) const
  {
    return build(std::string(class_name));
  }

  /// Look for a path in 'v', path can either be 'abs path' or 'rel path', one and only one
  /// should be provided
  static std::string readFilePath(const Json::Value& v, const std::string& dir_path)
  {
    std::string abs_path, rel_path;
    rhoban_utils::tryRead(v, "abs path", &abs_path);
    rhoban_utils::tryRead(v, "rel path", &rel_path);
    if (abs_path != "" && rel_path != "")
      throw JsonParsingError(DEBUG_INFO + " both 'abs path' and 'rel path' have been provided");
    else if (abs_path == "" && rel_path == "")
      throw JsonParsingError(DEBUG_INFO + " none of 'abs path' and 'rel path' have been provided");
    else if (abs_path != "")
      return abs_path;
    else
      return dir_path + rel_path;
  }

  /// Similar to readFilePath except it does nothing if neither 'abs path' nor 'rel path' is provided
  static void tryReadFilePath(const Json::Value& v, const std::string& dir_path, std::string* file_path)
  {
    if (v.isNull())
      throw JsonParsingError(DEBUG_INFO + "Trying to build from a NULL value");
    if (!v.isObject())
      throw JsonParsingError(DEBUG_INFO + "Trying to build from a non-object value");
    if (v.isMember("abs path") || v.isMember("rel path"))
      *file_path = readFilePath(v, dir_path);
  }

  /// When building from a node, the expected format of the json is the following:
  /// - String initialization
  ///   - "class name" : <object type> ,
  ///   - "content:" : ...
  /// or absolute path initialization:
  ///   - "abs path" : ...
  /// or relative path initializaion:
  ///   - "rel path" : ...
  std::unique_ptr<T> build(const Json::Value& v, const std::string& dir_path = "./") const
  {
    // Checking node validity
    if (v.isNull())
    {
      throw JsonParsingError("Factory::build: Trying to build from a NULL value");
    }
    if (!v.isObject())
    {
      throw JsonParsingError("Factory::build: Trying to build from a non-object value");
    }
    // Trying the three possible ways to read the document
    std::string file_path, class_name;
    tryReadFilePath(v, dir_path, &file_path);
    rhoban_utils::tryRead(v, "class name", &class_name);
    // Treating cases where multiple or no methods have been provided
    if (class_name != "" && file_path != "")
    {
      throw JsonParsingError("Factory::build: 'path' and 'class name' are incompatible");
    }
    if (class_name == "" && file_path == "")
    {
      throw JsonParsingError("Factory::build: requiring 'abs path', 'rel path' or 'class name'");
    }
    // Initializing and throwing error if two methods are provided
    if (file_path != "")
    {
      return buildFromJsonFile(file_path);
    }
    // Getting content
    if (!v.isMember("content"))
    {
      throw JsonParsingError("Factory::build: no 'content' found");
    }
    std::unique_ptr<T> obj;
    try
    {
      obj = getBuilder(class_name)(v["content"], dir_path);
    }
    catch (const JsonParsingError& exc)
    {
      throw JsonParsingError(std::string(exc.what()) + " in 'content'");
    }
    return obj;
  }

  std::unique_ptr<T> build(int id) const
  {
    return getBuilder(id)();
  }

  /// path: path to xml_file
  std::unique_ptr<T> buildFromJsonFile(const std::string& path) const
  {
    // Parse json
    /// Build:
    std::string dir_path = getDirName(path);
    std::unique_ptr<T> obj;
    try
    {
      Json::Value json_content = file2Json(path);
      obj = build(json_content, dir_path);
    }
    catch (const JsonParsingError& exc)
    {
      throw JsonParsingError(std::string(exc.what()) + " when building from file '" + path + "'");
    }
    return obj;
  }

  /// Throw a JsonParsingError, if value is not an object containing the given key,
  /// otherwise, build the object
  /// @see JsonSerializable::toFactoryJson
  std::unique_ptr<T> read(const Json::Value& value, const std::string& key, const std::string& dir_path)
  {
    if (!value.isObject())
    {
      throw JsonParsingError("Non object node when trying to read from a factory");
    }
    if (!value.isMember(key))
    {
      throw JsonParsingError("Could not find member '" + key + "'");
    }
    std::unique_ptr<T> obj;
    try
    {
      obj = build(value[key], dir_path);
    }
    catch (const JsonParsingError& exc)
    {
      throw JsonParsingError(std::string(exc.what()) + " in '" + key + "'");
    }
    return obj;
  }

  /// Fill 'ptr' with a generated object with the given value as argument
  /// If value is null or key is not found:
  /// - no error is thrown and 'ptr' content is not modified
  /// If key is found but content cannot be parsed:
  /// - an error is still raised and the 'ptr' is not overwritten
  /// If key is found and content is parsed properly:
  /// - 'ptr' is overwritten (and thus, old value is deleted according to unique_ptr)
  void tryRead(const Json::Value& value, const std::string& key, const std::string& dir_path, std::unique_ptr<T>* ptr)
  {
    if (!value.isObject())
    {
      return;
    }
    if (!value.isMember(key))
    {
      return;
    }
    *ptr = build(value[key], dir_path);
  }

  std::vector<std::unique_ptr<T>> readVector(const Json::Value& value, const std::string& key,
                                             const std::string& dir_path)
  {
    // Checks: object, member then array
    if (!value.isObject())
    {
      throw JsonParsingError("Non object node when trying to read from a factory");
    }
    if (!value.isMember(key))
    {
      throw JsonParsingError("Could not find member '" + key + "'");
    }
    const Json::Value key_value = value[key];
    if (!key_value.isArray())
    {
      throw JsonParsingError("Member '" + key + "' does not contain an array");
    }
    // Building internal objects
    std::vector<std::unique_ptr<T>> result;
    for (Json::ArrayIndex i = 0; i < key_value.size(); i++)
    {
      result.push_back(build(key_value[i], dir_path));
    }
    return result;
  }

  /// Try to read a vector with the given key in a json Value
  /// throw exceptions if:
  /// - value is not an object
  /// - an exception is thrown while building one of the element
  /// Do not throw exception if key does not exist (in this case res is not modified)
  void tryReadVector(const Json::Value& value, std::string& key, const std::string& dir_path,
                     std::vector<std::unique_ptr<T>>& res)
  {
    // Checks: object, member then array
    if (!value.isObject())
    {
      throw JsonParsingError("Non object node when trying to read from a factory");
    }
    if (!value.isMember(key))
    {
      return;
    }
    res = readVector(value, key, dir_path);
  }

  std::map<std::string, std::unique_ptr<T>> readMap(const Json::Value& value, const std::string& map_key,
                                                    const std::string& dir_path)
  {
    // Checks: object, member then array
    if (!value.isObject())
    {
      throw JsonParsingError("Non object node when trying to read from a factory");
    }
    if (!value.isMember(map_key))
    {
      throw JsonParsingError("Could not find member '" + map_key + "'");
    }
    const Json::Value map_v = value[map_key];
    if (!map_v.isObject())
    {
      throw JsonParsingError("readMap: Member '" + map_key + "' does not contain an object");
    }
    // Building internal objects
    std::map<std::string, std::unique_ptr<T>> result;
    for (Json::ValueConstIterator it = map_v.begin(); it != map_v.end(); it++)
    {
      const std::string& key = it.name();
      try
      {
        result[key] = build(map_v[key], dir_path);
      }
      catch (const JsonParsingError& exc)
      {
        throw JsonParsingError(std::string(exc.what()) + " in '" + key + "' in '" + map_key + "'");
      }
    }
    return result;
  }

  /// Return the number of bytes read
  int read(std::istream& in, std::unique_ptr<T>& ptr)
  {
    int bytes_read = 0;
    int id;
    bytes_read += rhoban_utils::read<int>(in, &id);
    int builder_bytes_read = 0;
    ptr = getStreamBuilder(id)(in, &builder_bytes_read);
    bytes_read += builder_bytes_read;
    return bytes_read;
  }

  /// Return the number of bytes read
  int loadFromFile(const std::string& filename, std::unique_ptr<T>& ptr)
  {
    std::ifstream in(filename, std::ios::binary);
    if (!in)
    {
      std::ostringstream oss;
      oss << "Failed to open '" << filename << "' for binary reading";
      throw JsonParsingError(oss.str());
    }
    int bytes_read = read(in, ptr);
    in.close();
    return bytes_read;
  }

  /// Read a path in the Json::Value (see readFilePath) and build an object from the binary reading
  /// at the given path
  std::unique_ptr<T> loadBinaryFromPath(const Json::Value& v, const std::string& dir_name)
  {
    std::string file_path = readFilePath(v, dir_name);
    std::unique_ptr<T> result;
    loadFromFile(file_path, result);
    return std::move(result);
  }

  /// If a path is properly provided, build an object and place it in 'result'
  /// If no path is provided, value of 'result' is not changed
  void tryLoadBinaryFromPath(const Json::Value& v, const std::string& dir_name, std::unique_ptr<T>* result)
  {
    std::string file_path;
    tryReadFilePath(v, dir_name, &file_path);
    if (file_path != "")
      loadFromFile(file_path, *result);
  }
  /// Convert an empty builder to a Json builder
  static JsonBuilder toJsonBuilder(Builder builder, bool parse_json)
  {
    return [builder, parse_json](const Json::Value& v, const std::string& dir) -> std::unique_ptr<T> {
      std::unique_ptr<T> object(builder());
      if (v.isObject() && parse_json)
        object->fromJson(v, dir);
      return std::move(object);
    };
  }

  /// Convert an empty builder to a stream builder
  static StreamBuilder toStreamBuilder(Builder builder)
  {
    return [builder](std::istream& in, int* nb_bytes_read) -> std::unique_ptr<T> {
      std::unique_ptr<T> object(builder());
      int tmp = object->read(in);
      if (nb_bytes_read != nullptr)
        *nb_bytes_read = tmp;
      return std::move(object);
    };
  }

  /// Send an error if a builder for the given class_name is already registered
  /// - Automatically transform the builder in JsonBuilder
  void registerBuilder(const std::string& class_name, Builder builder, bool parse_json = true)
  {
    registerBuilder(class_name, toJsonBuilder(builder, parse_json));
  }

  /// Send an error if a builder for the given class_name is already registered
  void registerBuilder(const std::string& class_name, JsonBuilder builder)
  {
    if (json_builders.count(class_name) != 0)
      throw std::runtime_error("Factory: registering a class named '" + class_name + "' while it already exists");
    json_builders[class_name] = builder;
  }

  /// Send an error if a builder for the given id is already registered
  /// if autocreate_streambuilder is true, then it also create and register a streambuilder
  void registerBuilder(int id, Builder builder, bool autocreate_streambuilder = true)
  {
    if (autocreate_streambuilder)
    {
      registerBuilder(id, toStreamBuilder(builder));
    }
    if (builders_by_id.count(id) != 0)
    {
      std::ostringstream oss;
      oss << "Factory: registering a class with id '" << id << "' while it is already used";
      listBuilders(oss);
      throw std::runtime_error(oss.str());
    }
    builders_by_id[id] = builder;
  }

  /// Send an error if a builder for the given id is already registered
  void registerBuilder(int id, StreamBuilder builder)
  {
    if (stream_builders_by_id.count(id) != 0)
    {
      std::ostringstream oss;
      oss << "Factory: registering a class with id '" << id << "' while it is already used";
      throw std::runtime_error(oss.str());
    }
    stream_builders_by_id[id] = builder;
  }

  template <class ChildType>
  void importJsonBuilders(const Factory<ChildType>& other_factory)
  {
    for (const auto& entry : other_factory.getJsonBuilders())
    {
      registerBuilder(entry.first, [entry](const Json::Value& v, const std::string& dir) {
        return std::unique_ptr<T>(entry.second(v, dir).release());
      });
    }
  }

  const std::map<std::string, JsonBuilder> getJsonBuilders() const
  {
    return json_builders;
  }

  /// List all the known builders to the stream
  void listBuilders(std::ostream& out) const
  {
    out << "Json Builders: " << std::endl;
    for (const auto& entry : json_builders)
    {
      out << "\t" << entry.first << std::endl;
    }
    out << "Default Builders: " << std::endl;
    for (const auto& entry : builders_by_id)
    {
      out << "\t" << entry.first << std::endl;
    }
    out << "Stream Builders: " << std::endl;
    for (const auto& entry : stream_builders_by_id)
    {
      out << "\t" << entry.first << std::endl;
    }
  }

private:
  /// Contains a mapping from class names to json builders
  std::map<std::string, JsonBuilder> json_builders;
  /// Contains a mapping from id to builders
  std::map<int, Builder> builders_by_id;
  /// Contains a mapping from id to stream builders
  std::map<int, StreamBuilder> stream_builders_by_id;
};

}  // namespace rhoban_utils
