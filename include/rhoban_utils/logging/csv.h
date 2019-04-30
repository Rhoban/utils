#ifndef _RHOBAN_CSV_H
#define _RHOBAN_CSV_H

#include <iostream>
#include <fstream>
#include <map>
#include <string>

namespace rhoban_utils
{
class CSV
{
public:
  CSV();

  void open(std::string filename, char _separator = ',');
  void push(std::string column, double value);
  void newLine();
  void close();

protected:
  std::ofstream ofs;
  bool header;
  std::map<std::string, int> columns;
  std::map<int, std::string> columnIndexes;
  std::map<int, double> values;

  char separator;

  void produceHeader();
};
}  // namespace rhoban_utils

#endif
