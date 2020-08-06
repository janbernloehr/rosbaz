#pragma once

#include <cstdint>

namespace rosbaz
{
namespace bag_parsing
{
/// Describes the position and size of a message record within a chunk record.
struct MessageRecordInfo
{
  MessageRecordInfo(uint64_t _offset, uint32_t _data_size) : offset{ _offset }, data_size{ _data_size }
  {
  }

  uint64_t offset{ 0 };  // relative to data section of chunk record
  uint32_t data_size{ 0 };
};
}  // namespace bag_parsing
}  // namespace rosbaz