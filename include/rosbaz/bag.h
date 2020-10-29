#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

#include <rosbag/structures.h>

#include "rosbaz/bag_parsing/chunk_ext.h"
#include "rosbaz/bag_parsing/header.h"
#include "rosbaz/bag_parsing/record.h"

namespace rosbaz
{
namespace io
{
class IReader;
}  // namespace io

/// Implements a subset of the functionality provided by rosbag::Bag offloading
/// the actual data reading to an implementation of rosbaz::io::IReader provided
/// by the user.
class Bag
{
public:
  friend struct View;
  friend struct MessageInstance;

  /// Create a bag instance using the given reader.
  static Bag read(std::shared_ptr<rosbaz::io::IReader> reader, bool read_chunk_indices = true);

  std::vector<const rosbag::ConnectionInfo*> getConnections() const;

  std::uint32_t getMessageCountForConnectionId(uint32_t id) const;

  std::uint64_t getSize() const
  {
    return file_size_;
  }
  std::uint32_t getChunkCount() const
  {
    return chunk_count_;
  }

  ros::Time getBeginTime() const;
  ros::Time getEndTime() const;

private:
  explicit Bag(std::shared_ptr<rosbaz::io::IReader> reader);

  void parseFileHeaderRecord(const rosbaz::bag_parsing::Record& file_header_record);

  void parseFileTail(rosbaz::DataSpan bag_tail);

  /// Parses the index records contained in \p chunk_index following a chunk record described by \p chunk_ext. This
  /// populates \p connection_indexes_ and also sets \p message_records of the given \p chunk_ext.
  ///
  /// The implementation has to be thread safe since it may be invoked simultaneously for multiple chunks.
  void parseIndexSection(std::mutex& sync, rosbaz::bag_parsing::ChunkExt& chunk_ext, rosbaz::DataSpan chunk_index,
                         const uint64_t index_offset);

  /// Reads the chunk header for the given \p chunk_info, extracts the position of and reads the index sections which
  /// subsequently is analyzed by \p parseIndexSection.
  ///
  /// The implementation has to be thread safe since it may be invoked simultaneously for multiple chunks.
  void parseChunkInfo(std::mutex& sync, rosbaz::io::IReader& reader, const rosbag::ChunkInfo& chunk_info,
                      uint64_t next_chunk_pos);

  /// Fills \p connection_indexes_ and \p chunks_.
  ///
  /// The implementation may read data for the present chunks simultaneously.
  void parseChunkIndices(rosbaz::io::IReader& reader);

  mutable std::shared_ptr<rosbaz::io::IReader> reader_{};

  std::unordered_map<uint32_t, rosbag::ConnectionInfo> connections_{};

  std::uint32_t bag_revision_{ 0 };

  std::uint32_t chunk_count_{ 0 };
  std::uint32_t connection_count_{ 0 };

  std::uint64_t file_header_pos_{ 0 };
  std::uint64_t index_data_pos_{ 0 };

  std::uint64_t file_size_{ 0 };

  std::vector<rosbag::ChunkInfo> chunk_infos_{};

  bool chunk_indices_parsed_ = false;

  std::unordered_map<uint32_t, std::multiset<rosbag::IndexEntry>> connection_indexes_{};
  std::vector<rosbaz::bag_parsing::ChunkExt> chunks_{};
};
}  // namespace rosbaz