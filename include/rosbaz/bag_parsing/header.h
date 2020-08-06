#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>

#include "rosbaz/common.h"

namespace rosbaz
{
namespace bag_parsing
{
/// Represents a header of the rosbag format - see
/// http://wiki.ros.org/Bags/Format/2.0
struct Header
{
  std::uint8_t op{ 0 };
  std::unordered_map<std::string, rosbaz::DataSpan> fields{};


  static Header parse(rosbaz::DataSpan source);
};
}  // namespace bag_parsing
}  // namespace rosbaz