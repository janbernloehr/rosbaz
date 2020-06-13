#pragma once

#include <cstdint>

namespace rosbaz
{
namespace bag_parsing
{
/// Describes the position and size of a message record within a chunk record.
struct MessageRecordInfo
{
  uint64_t offset;  // relative to data section of chunk record
  uint32_t data_size;
};
}  // namespace bag_parsing
}  // namespace rosbaz