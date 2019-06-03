#pragma once

#include <ostream>
#include <string>
#include <vector>
#include <list>
#include <stdexcept>
#include <ctime>

#define DEBUG_INFO                                                                                                     \
  (std::string(__FUNCTION__) + ":" + rhoban_utils::getBaseName(__FILE__) + ":" + std::to_string(__LINE__) + ": ")

namespace rhoban_utils
{
std::vector<std::string>& split(const std::string& s, char delim, std::vector<std::string>& elems);

void file_put_contents(std::string path, std::string contents);

void replaceAll(std::string& str, const std::string& from, const std::string& to);
bool strContains(const std::string& str, const std::string& other);

bool file_exists(const std::string& name);

/// Return the name of the file at the given path:
/// e.g getBaseName("toto/file.cpp") returns "file.cpp"
std::string getBaseName(const std::string& path);

/// Return the name of the path to the directory containing the given file:
/// e.g getDirName("toto/file.cpp") returns "toto"
/// Remark: getDirName("file.cpp") returns "./"
std::string getDirName(const std::string& path);

/**
 * Return the name of current host, throws runtime error on failure
 */
std::string getHostName();

/// Append vector 'v' to the stream 'out', ensuring the following content is
/// added (spaces are present only for readability):
/// prefix v[0] separator v[1] ... separator v[n] suffix
template <typename T>
void appendVector(const std::vector<T> v, std::ostream& out, const std::string& separator = ',',
                  const std::string& prefix = "[", const std::string& suffix = "]")
{
  out << prefix;
  for (size_t idx = 0; idx < v.size(); idx++)
  {
    out << v[idx];
    if (idx < v.size() - 1)
      out << separator;
  }
  out << suffix;
}

}  // namespace rhoban_utils
