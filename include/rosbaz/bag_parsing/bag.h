#pragma once

#include <cstdint>
#include <unordered_map>
#include <vector>

#include <rosbag/structures.h>

#include "rosbaz/bag_parsing/chunk_ext.h"
#include "rosbaz/bag_parsing/header.h"
#include "rosbaz/bag_parsing/record.h"

namespace rosbaz {
namespace io {
class IReader;
}

namespace bag_parsing {

/// Implements a subset of the functionality provided by rosbag::Bag offloading
/// the actual data reading to an implementation of rosbaz::io::IReader provided
/// by the user.
class AzBag {
public:
  friend class View;
  friend class MessageInstance;

  /// Create a bag instance using the given reader.
  static AzBag read(rosbaz::io::IReader &reader,
                    bool read_chunk_indices = true);

  std::vector<const rosbag::ConnectionInfo *> getConnections() const;

  std::uint32_t getMessageCountForConnectionId(uint32_t id) const;

  std::uint64_t getFileSize() const { return file_size_; }
  std::uint32_t getChunkCount() const { return chunk_count_; }

  ros::Time getBeginTime() const;
  ros::Time getEndTime() const;

private:
  AzBag() = default;

  void parseFileHeaderRecord(const Record &file_header_record);

  void parseFileTail(rosbaz::DataSpan bag_tail);

  /// Parses the indexes following a chunk record and populates
  /// connection_indexes_; also sets message_records of the given ChunkExt.
  void parseIndexSection(ChunkExt &chunk_ext, rosbaz::DataSpan chunk_index,
                         const uint64_t index_offset);

  /// Fills connection_indexes_ and chunks_.
  void parseChunkIndices(rosbaz::io::IReader &reader);

  std::unordered_map<uint32_t, rosbag::ConnectionInfo> connections_;

  std::uint32_t chunk_count_;
  std::uint32_t connection_count_;

  std::uint64_t file_header_pos_;
  std::uint64_t index_data_pos_;

  std::uint64_t file_size_;

  std::vector<rosbag::ChunkInfo> chunk_infos_;

  bool chunk_indices_parsed_ = false;

  std::unordered_map<uint32_t, std::multiset<rosbag::IndexEntry>>
      connection_indexes_;
  std::vector<ChunkExt> chunks_;
};
} // namespace bag_parsing
} // namespace rosbaz