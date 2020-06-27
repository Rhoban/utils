#include "rhoban_utils/tables/string_table.h"

#include "rhoban_utils/io_tools.h"
#include "rhoban_utils/util.h"

#include <fstream>

namespace rhoban_utils
{
StringTable::StringTable()
{
}

StringTable::StringTable(const std::vector<std::string>& column_names_) : column_names(column_names_)
{
  for (const std::string& key : column_names)
  {
    data[key] = Column();
  }
}
StringTable::StringTable(const std::vector<std::string>& column_names_, const std::map<std::string, Column>& data_)
  : column_names(column_names_), data(data_)
{
  if (column_names.size() != data.size())
  {
    throw std::logic_error(DEBUG_INFO + " size mismatch");
  }
  for (const std::string& col_name : column_names)
  {
    if (data.count(col_name) == 0)
    {
      throw std::logic_error(DEBUG_INFO + " data does not contain column '" + col_name + "'");
    }
  }
}

StringTable StringTable::buildFromFile(const std::string& file_path, char separator, bool has_header)
{
  return buildFromString(file2string(file_path), separator, has_header);
}

StringTable StringTable::buildFromString(const std::string& str, char separator, bool has_header)
{
  // Convert string to lines
  std::vector<std::string> lines;
  split(str, '\n', lines);
  // If there are no lines throw error
  if (lines.size() == 0)
  {
    throw std::runtime_error(DEBUG_INFO + " no lines in string");
  }
  // Find out column names or generate them
  std::vector<std::string> column_names;
  int first_line_idx = 0;
  if (has_header)
  {
    first_line_idx = 1;
    split(lines[0], separator, column_names);
  }
  else
  {
    // Number of columns is defined by first line columns
    std::vector<std::string> tmp;
    split(lines[0], separator, tmp);
    for (size_t col = 0; col < tmp.size(); col++)
    {
      column_names.push_back("col_" + std::to_string(col));
    }
  }
  // Fill content
  std::map<std::string, Column> data;
  for (const std::string& col : column_names)
  {
    data[col] = std::vector<std::string>();
  }

  for (size_t line_idx = first_line_idx; line_idx < lines.size(); line_idx++)
  {
    const std::string& line = lines[line_idx];
    std::vector<std::string> values;
    split(line, separator, values);
    if (values.size() != column_names.size())
    {
      throw std::runtime_error(DEBUG_INFO + " invalid row " + std::to_string(line_idx) + ": expecting " +
                               std::to_string(column_names.size()) + " columns, " + std::to_string(values.size()) +
                               " received");
    }
    for (size_t col = 0; col < values.size(); col++)
    {
      data[column_names[col]].push_back(values[col]);
    }
  }
  return StringTable(column_names, data);
}

void StringTable::writeFile(const std::string& file_path, const std::string& separator) const
{
  std::ofstream out(file_path);
  if (out.bad())
  {
    throw std::logic_error(DEBUG_INFO + "Failed to open file: '" + file_path + "'");
  }
  dumpHeader(out, separator);
  dumpToStream(out, separator);
  // write content
}

size_t StringTable::nbCols() const
{
  return column_names.size();
}

size_t StringTable::nbRows() const
{
  if (nbCols() == 0)
    return 0;
  return getColumn(column_names[0]).size();
}

const std::vector<std::string>& StringTable::getColumnNames() const
{
  return column_names;
}

const StringTable::Column& StringTable::getColumn(const std::string& column_name) const
{
  return data.at(column_name);
}

std::map<std::string, std::string> StringTable::getRow(size_t row) const
{
  if (row >= nbRows())
  {
    throw std::out_of_range(DEBUG_INFO + "asking for row " + std::to_string(row) + " while table has " +
                            std::to_string(nbRows()) + " rows (indices start at 0)");
  }
  std::map<std::string, std::string> result;
  for (const std::string col_name : column_names)
  {
    result[col_name] = data.at(col_name)[row];
  }
  return result;
}

void StringTable::insertRow(const std::map<std::string, std::string>& row)
{
  if (row.size() != column_names.size())
  {
    throw std::out_of_range(DEBUG_INFO + "size mismatch: row: " + std::to_string(row.size()) +
                            " column_names: " + std::to_string(column_names.size()));
  }
  // Checking names match
  for (const auto& entry : row)
  {
    if (data.count(entry.first) == 0)
    {
      throw std::out_of_range(DEBUG_INFO + "row: '" + entry.first + "' does not belong to column_names");
    }
  }
  for (const auto& entry : row)
  {
    data[entry.first].push_back(entry.second);
  }
  if (isStreaming())
  {
    dumpRow(output, nbRows() - 1);
  }
}

void StringTable::clearData()
{
  for (const std::string& name : column_names)
  {
    data[name].clear();
  }
}

void StringTable::dump(const std::string& file_path)
{
  startStreaming(file_path);
  endStreaming();
}

void StringTable::startStreaming(const std::string& file_path)
{
  if (column_names.size() == 0)
    throw std::logic_error("Cannot open stream without column names");
  if (isStreaming())
    throw std::logic_error("Stream is already opened");
  output.open(file_path);
  if (!output.good())
    throw std::runtime_error("Failed to open stream at '" + file_path + "'");
  dumpHeader(output);
  dumpToStream(output);
}

void StringTable::endStreaming()
{
  if (!isStreaming())
    throw std::logic_error("Not streaming");
  output.close();
}

bool StringTable::isStreaming() const
{
  return output.is_open();
}

void StringTable::dumpHeader(std::ostream& out, const std::string& separator) const
{
  appendVector(column_names, out, separator, "", "");
  out << std::endl;
}

void StringTable::dumpToStream(std::ostream& out, const std::string& separator) const
{
  for (size_t row = 0; row < nbRows(); row++)
  {
    dumpRow(out, row, separator);
  }
}

void StringTable::dumpRow(std::ostream& out, size_t row, const std::string& separator) const
{
  for (size_t col = 0; col < nbCols(); col++)
  {
    out << getColumn(column_names[col])[row];
    if (col < nbCols() - 1)
      out << separator;
  }
  out << std::endl;
}

}  // namespace rhoban_utils
