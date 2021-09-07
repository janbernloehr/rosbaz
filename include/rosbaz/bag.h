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

namespace compression
{
enum CompressionType
{
  Uncompressed = 0,
  BZ2 = 1,
};
}  // namespace compression
using CompressionType = compression::CompressionType;

namespace bagmode
{
enum BagMode
{
  Write = 1,
  Read = 2,
  Append = 4
};
}  // namespace bagmode
using BagMode = bagmode::BagMode;

class BagSatistics;

/// Implements a subset of the functionality provided by rosbag::Bag offloading
/// the actual data reading to an implementation of rosbaz::io::IReader provided
/// by the user.
class Bag
{
public:
  friend struct View;
  friend struct MessageInstance;
  friend class BagSatistics;

  /// Create a bag instance using the given reader.
  static Bag read(std::shared_ptr<rosbaz::io::IReader> reader, bool read_chunk_indices = true);

  std::vector<const rosbag::ConnectionInfo*> getConnections() const;

  /// Get the filepath of the bag.
  std::string getFilePath() const;

  /// Get the mode the bag is in.
  BagMode getMode() const;

  /// Get the major-version of the open bag file.
  uint32_t getMajorVersion() const;

  /// Get the minor-version of the open bag file.
  uint32_t getMinorVersion() const;

  /// Get the current size of the bag file (a lower bound)
  std::uint64_t getSize() const;

  /// Get the total number of chunks in the bag file.
  std::uint32_t getChunkCount() const;

  /// Get the threshold for creating new chunks.
  uint32_t getChunkThreshold() const;

  /// Get the compression method to use for writing chunks.
  CompressionType getCompression() const;

  ros::Time getBeginTime() const;
  ros::Time getEndTime() const;

private:
  explicit Bag(std::shared_ptr<rosbaz::io::IReader> reader);

  void parseFileHeaderRecord(const rosbaz::bag_parsing::Record& file_header_record);

  void parseFileTail(rosbaz::DataSpan bag_tail);

  /// Parses the index records contained in \p chunk_index following a chunk record described by \p chunk_ext. This
  /// populates \p index_entries and \p message_records of the given \p chunk_ext.
  ///
  /// The implementation has to be thread safe since it may be invoked simultaneously for multiple chunks.
  void parseIndexSection(rosbaz::bag_parsing::ChunkExt& chunk_ext, rosbaz::DataSpan chunk_index,
                         const uint64_t index_offset);

  /// Reads the chunk header for the given \p chunk_info, extracts the position of and reads the index sections which
  /// subsequently is analyzed by \p parseIndexSection.
  ///
  /// The implementation has to be thread safe since it may be invoked simultaneously for multiple chunks.
  void parseChunkInfo(std::mutex& sync, rosbaz::io::IReader& reader, const rosbag::ChunkInfo& chunk_info,
                      uint64_t next_chunk_pos);

  /// Fills \p connection_indexes_ and \p chunk_exts_.
  ///
  /// The implementation may read data for the present chunks simultaneously.
  void parseChunkIndices(rosbaz::io::IReader& reader);

  mutable std::shared_ptr<rosbaz::io::IReader> reader_{};

  // rosbag uses a map<uint32_t, ConnectionInfo*>. Since only parseFileTail can modify connections_
  // it is safe to store the ConnectionInfo in the map. Only expansion would invalidate the addresses.
  std::unordered_map<uint32_t, rosbag::ConnectionInfo> connections_{};

  std::uint32_t bag_revision_{ 0 };

  std::uint32_t chunk_count_{ 0 };
  std::uint32_t connection_count_{ 0 };

  std::uint64_t file_header_pos_{ 0 };
  std::uint64_t index_data_pos_{ 0 };

  std::uint64_t file_size_{ 0 };

  // plain rosbag chunk infos
  std::vector<rosbag::ChunkInfo> chunks_{};
  // extended chunk info needed for rosbaz
  std::vector<rosbaz::bag_parsing::ChunkExt> chunk_exts_{};
  std::unordered_map<uint64_t, const rosbaz::bag_parsing::ChunkExt*> chunk_exts_lookup_{};

  bool chunk_indices_parsed_ = false;

  // rosbag uses a map. Since there are as many indices as chunk infos, it is safe to reserve once.
  std::unordered_map<uint32_t, std::multiset<rosbag::IndexEntry>> connection_indexes_{};
};
}  // namespace rosbaz