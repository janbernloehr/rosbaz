#pragma once

#include <cstdint>
#include <unordered_map>
#include <vector>

#include <rosbag/structures.h>

#include "rosbaz/bag_parsing/message_record_info.h"

namespace rosbaz
{
namespace bag_parsing
{
struct IndexEntryExt
{
  IndexEntryExt(uint32_t _connection_id, const rosbag::IndexEntry& _index_entry)
    : connection_id{ _connection_id }, index_entry{ _index_entry }
  {
  }

  uint32_t connection_id;
  rosbag::IndexEntry index_entry;
};

/// Provides extended information for a given chunk (see
/// http://wiki.ros.org/Bags/Format/2.0).
///
/// In addition to the rosbag::ChunkInfo describing the offset of the given
/// chunk in the bag file, we provide offset and size of the data section of the
/// chunk, offset and size of the index section following the chunk, as well as
/// offsets and sizes of all message records within the data section together with all
/// index entries of the chunks.
struct ChunkExt
{
  ChunkExt(const rosbag::ChunkInfo& _chunk_info) : chunk_info(_chunk_info)
  {
  }

  const rosbag::ChunkInfo& chunk_info;
  rosbag::ChunkHeader chunk_header{};
  std::uint64_t data_offset{ 0 };  // absolute
  std::uint32_t data_size{ 0 };

  std::unordered_map<uint64_t, MessageRecordInfo> message_records{};

  std::vector<IndexEntryExt> index_entries{};
};
}  // namespace bag_parsing
}  // namespace rosbaz
