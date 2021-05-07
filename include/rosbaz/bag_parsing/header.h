#pragma once

#include <cstdint>
#include <string>
#include <sstream>
#include <map>

#include "rosbaz/common.h"
#include "rosbaz/exceptions.h"
#include "rosbaz/io/io_helpers.h"

namespace rosbaz
{
namespace bag_parsing
{
/// Represents a header of the rosbag format - see
/// http://wiki.ros.org/Bags/Format/2.0
struct Header
{
  std::uint8_t op{ 0 };
  // in this scenario, a map is faster than an unordered map
  std::map<std::string, rosbaz::DataSpan> fields{};

  template <class T>
  T read_field_little_endian(const std::string& field_name) const
  {
    if (fields.count(field_name) == 0)
    {
      std::stringstream msg;
      msg << "Missing field " << field_name << " in header";
      throw rosbaz::UnsupportedRosBagException(msg.str());
    }

    return rosbaz::io::read_little_endian<T>(fields.at(field_name));
  }

  static Header parse(rosbaz::DataSpan source);
};
}  // namespace bag_parsing
}  // namespace rosbaz