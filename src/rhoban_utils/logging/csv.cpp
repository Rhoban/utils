#include "rhoban_utils/logging/csv.h"

namespace rhoban_utils
{
CSV::CSV()
{
  separator = ',';
}

void CSV::open(std::string filename, char _separator)
{
  ofs.open(filename.c_str());
  ofs.precision(10);
  header = false;
  separator = _separator;
}

void CSV::push(std::string column, double value)
{
  if (columns.count(column))
  {
    values[columns[column]] = value;
  }
  else if (!header)
  {
    int index = columns.size();
    columns[column] = index;
    columnIndexes[index] = column;
    values[index] = value;
  }
}

void CSV::newLine()
{
  if (!header)
  {
    produceHeader();
    header = true;
  }

  for (unsigned int index = 0; index < columns.size(); index++)
  {
    ofs << values[index];
    if (index != columns.size() - 1)
      ofs << separator;
  }
  ofs << std::endl;
  ofs.flush();
}

void CSV::produceHeader()
{
  for (unsigned int index = 0; index < columnIndexes.size(); index++)
  {
    ofs << columnIndexes[index];
    if (index != columnIndexes.size() - 1)
    {
      ofs << separator;
    }
    else
    {
      ofs << std::endl;
    }
  }
}

void CSV::close()
{
  ofs.close();
}

}  // namespace rhoban_utils
