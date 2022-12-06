#pragma once

#include "rosbaz/common.h"

#include <cstdint>

namespace rosbaz
{
namespace bag_parsing
{
/// Represents a record of the rosbag format - see
/// http://wiki.ros.org/Bags/Format/2.0
struct Record
{
  std::uint32_t header_length{ 0 };
  rosbaz::DataSpan header{};
  std::uint32_t data_length{ 0 };
  rosbaz::DataSpan data{};

  std::uint32_t total_size() const
  {
    return static_cast<std::uint32_t>(sizeof(std::uint32_t)) + header_length +
           static_cast<std::uint32_t>(sizeof(std::uint32_t)) + data_length;
  }

  static Record parse(rosbaz::DataSpan source);
};
}  // namespace bag_parsing
}  // namespace rosbaz