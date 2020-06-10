#pragma once

#include <cstdint>

#include <rosbag/structures.h>

#include "rosbaz/bag_parsing/message_record_info.h"

namespace rosbaz {

namespace bag_parsing {
/// Provides extended information for a given chunk (see
/// http://wiki.ros.org/Bags/Format/2.0).
///
/// In addition to the rosbag::ChunkInfo describing the offset of the given
/// chunk in the bag file, we provide offset and size of the data section of the
/// chunk, offset and size of the index section following the chunk, as well as
/// offsets and sizes of all message records within the data section.
struct ChunkExt {
  ChunkExt(const rosbag::ChunkInfo &_chunk_info,
           const rosbag::ChunkHeader &_chunk_header, std::uint64_t _data_offset,
           std::uint32_t _data_size, std::uint64_t _index_offset,
           std::uint32_t _index_size)
      : chunk_info(_chunk_info), chunk_header(_chunk_header),
        data_offset(_data_offset), data_size(_data_size),
        index_offset(_index_offset), index_size(_index_size) {}

  const rosbag::ChunkInfo &chunk_info;
  rosbag::ChunkHeader chunk_header;
  std::uint64_t data_offset; // absolute
  std::uint32_t data_size;

  std::uint64_t index_offset; // absolute
  std::uint32_t index_size;

  std::vector<MessageRecordInfo> message_records;
};
} // namespace bag_parsing
} // namespace rosbaz