#pragma once

#include "ros/console.h"
#include "rosbaz/bag_writing/block_ext.h"
#include "rosbaz/bag_writing/conversion.h"
#include "rosbaz/common.h"
#include "rosbaz/exceptions.h"
#include "rosbaz/io/buffer.h"
#include "rosbaz/io/util.h"
#include "rosbaz/io/writer.h"

#include <boost/optional.hpp>
#include <boost/smart_ptr/make_shared_object.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <ros/console.h>
#include <ros/datatypes.h>
#include <ros/message_traits.h>
#include <ros/serialization.h>
#include <ros/time.h>
#include <rosbag/constants.h>
#include <rosbag/structures.h>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace ros
{
template <typename M>
class MessageEvent;
}  // namespace ros

namespace rosbaz
{
namespace bag_parsing
{
struct ChunkExt;
struct Record;
}  // namespace bag_parsing

namespace io
{
class IReader;
class IWriter;
class Block;
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

class BagStatistics;

/// Implements a subset of the functionality provided by rosbag::Bag offloading
/// the actual data reading to an implementation of rosbaz::io::IReader provided
/// by the user.
class Bag
{
public:
  friend struct View;
  friend struct MessageInstance;
  friend class BagStatistics;

  Bag(const Bag&) = default;
  Bag(Bag&&) = default;
  Bag& operator=(const Bag&) = default;
  Bag& operator=(Bag&&) = default;

  ~Bag() noexcept(false);

  /// Create a bag instance using the given reader.
  static Bag read(std::shared_ptr<rosbaz::io::IReader> reader, bool read_chunk_indices = true);

  /// Create a bag instance using the given writer.
  static Bag write(std::shared_ptr<rosbaz::io::IWriter> writer);

  /// Close the bag file
  void close();

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

  /// Set the threshold for creating new chunks
  void setChunkThreshold(uint32_t chunk_threshold);

  /// Get the compression method to use for writing chunks.
  ///
  /// This is not part of the official rosbag API.
  CompressionType getCompression() const;

  /// Get the earliest start time of all chunks of the bag file.
  ///
  /// This is not part of the official rosbag API.
  ros::Time getBeginTime() const;

  /// Get the latest end time of all chunks of the bag file.
  ///
  /// This is not part of the official rosbag API.
  ros::Time getEndTime() const;

  /// \return true if the bag does not have a valid index; otherwise false.
  ///
  /// This is not part of the official rosbag API.
  bool isUnindexed() const;

  /// Write a message into the bag file
  ///
  /// \param topic The topic name
  /// \param event The message event to be added
  ///
  /// Can throw BagIOException
  template <class T>
  void write(const std::string& topic, ros::MessageEvent<T> const& event);

  /// Write a message into the bag file
  ///
  /// \param topic The topic name
  /// \param time  Timestamp of the message
  /// \param msg   The message to be added
  /// \param connection_header  A connection header.
  ///
  /// Can throw BagIOException
  template <class T>
  void write(const std::string& topic, ros::Time const& time, T const& msg,
             boost::shared_ptr<ros::M_string> connection_header = boost::shared_ptr<ros::M_string>());

  /// Write a message into the bag file
  ///
  /// \param topic The topic name
  /// \param time  Timestamp of the message
  /// \param msg   The message to be added
  /// \param connection_header  A connection header.
  ///
  /// Can throw BagIOException
  template <class T>
  void write(const std::string& topic, ros::Time const& time, boost::shared_ptr<T const> const& msg,
             boost::shared_ptr<ros::M_string> connection_header = boost::shared_ptr<ros::M_string>());

  /// Write a message into the bag file
  ///
  /// \param topic The topic name
  /// \param time  Timestamp of the message
  /// \param msg   The message to be added
  /// \param connection_header  A connection header.
  ///
  /// Can throw BagIOException
  template <class T>
  void write(const std::string& topic, ros::Time const& time, boost::shared_ptr<T> const& msg,
             boost::shared_ptr<ros::M_string> connection_header = boost::shared_ptr<ros::M_string>());

private:
  explicit Bag(std::shared_ptr<rosbaz::io::IReader> reader);
  explicit Bag(std::shared_ptr<rosbaz::io::IWriter> writer);

  // Reading

  void parseFileHeaderRecord(const rosbaz::bag_parsing::Record& file_header_record);

  void parseFileTail(rosbaz::DataSpan bag_tail);

  /// Parses the index records contained in \p chunk_index following a chunk record described by \p chunk_ext. This
  /// populates \p index_entries and \p message_records of the given \p chunk_ext.
  ///
  /// The implementation has to be thread safe since it may be invoked simultaneously for multiple chunks.
  void parseIndexSection(rosbaz::bag_parsing::ChunkExt& chunk_ext, rosbaz::DataSpan chunk_index,
                         const uint64_t index_offset);

  /// Reads the chunk header for the given \p chunk_ext.chunk_info, extracts the position of and reads the index
  /// sections which subsequently is analyzed by \p parseIndexSection.
  ///
  /// The implementation has to be thread safe since it may be invoked simultaneously for multiple chunks.
  void parseChunkInfo(rosbaz::io::IReader& reader, rosbaz::bag_parsing::ChunkExt& chunk_ext, uint64_t next_chunk_pos);

  /// Fills \p connection_indexes_ and \p chunk_exts_.
  ///
  /// The implementation may read data for the present chunks simultaneously.
  void parseChunkIndices(rosbaz::io::IReader& reader);

  // Writing

  /// Returns the current offset in the chunk, relative to the start of the chunk record data.
  ///
  /// Should only be called after startWritingChunk() and before writing to this
  /// chunk concludes via stopWritingChunk() or indirectly via stopWriting() etc.
  uint32_t getChunkOffset() const;

  void writeVersion(rosbaz::io::Block& block);
  void writeFileHeaderRecord(rosbaz::io::Block& block);

  void writeConnectionRecord(rosbag::ConnectionInfo const* connection_info);
  template <class T>
  void writeMessageDataRecord(uint32_t conn_id, ros::Time const& time, T const& msg);
  void writeIndexRecords();
  void writeConnectionRecords();
  void writeChunkInfoRecords();
  void startWritingChunk(ros::Time time);
  void writeChunkHeader(CompressionType compression, uint32_t compressed_size, uint32_t uncompressed_size);
  void stopWritingChunk();

  uint32_t writeHeader(ros::M_string const& fields, boost::optional<size_t> offset = boost::none);
  uint32_t writeDataLength(uint32_t data_len, boost::optional<size_t> offset = boost::none);

  // This helper function actually does the write with an arbitrary serializable message
  template <class T>
  void doWrite(const std::string& topic, ros::Time const& time, T const& msg,
               boost::shared_ptr<ros::M_string> const& connection_header);

  void closeWrite();

  void startWriting();
  void stopWriting();

  // shared for reading and writing

  BagMode mode_;

  std::map<uint32_t, std::shared_ptr<rosbag::ConnectionInfo>> connections_{};

  std::uint32_t bag_revision_{ 0 };

  // plain rosbag chunk infos
  std::vector<rosbag::ChunkInfo> chunks_{};

  // for reading
  mutable std::shared_ptr<rosbaz::io::IReader> reader_{};

  std::uint32_t chunk_count_{ 0 };
  std::uint32_t connection_count_{ 0 };

  std::uint64_t file_header_pos_{ 0 };
  std::uint64_t index_data_pos_{ 0 };

  std::uint64_t file_size_{ 0 };

  // extended chunk info needed for rosbaz
  std::vector<std::shared_ptr<rosbaz::bag_parsing::ChunkExt>> chunk_exts_{};
  std::unordered_map<uint64_t, const rosbaz::bag_parsing::ChunkExt*> chunk_exts_lookup_{};

  bool chunk_indices_parsed_ = false;

  // rosbag uses a map. Since for reading there are as many indices as chunk infos, it is safe to reserve once.
  std::unordered_map<uint32_t, std::multiset<rosbag::IndexEntry>> connection_indexes_;

  // for writing
  mutable std::shared_ptr<rosbaz::io::IWriter> writer_{};

  std::map<std::string, uint32_t> topic_connection_ids_;
  std::map<ros::M_string, uint32_t> header_connection_ids_;

  uint32_t chunk_threshold_{ 768 * 1024 };  // 768KB chunks

  rosbag::ChunkInfo curr_chunk_info_;
  size_t curr_chunk_data_pos_{};
  std::map<uint32_t, std::multiset<rosbag::IndexEntry>> curr_chunk_connection_indexes_;

  std::shared_ptr<rosbaz::io::Block> header_block_;
  std::shared_ptr<rosbaz::io::Block> current_block_;
  rosbaz::io::Buffer record_buffer_;
};

// Templated method definitions

template <class T>
void Bag::write(const std::string& topic, ros::MessageEvent<T> const& event)
{
  doWrite(topic, event.getReceiptTime(), *event.getMessage(), event.getConnectionHeaderPtr());
}

template <class T>
void Bag::write(const std::string& topic, ros::Time const& time, T const& msg,
                boost::shared_ptr<ros::M_string> connection_header)
{
  doWrite(topic, time, msg, connection_header);
}

template <class T>
void Bag::write(const std::string& topic, ros::Time const& time, boost::shared_ptr<T const> const& msg,
                boost::shared_ptr<ros::M_string> connection_header)
{
  doWrite(topic, time, *msg, connection_header);
}

template <class T>
void Bag::write(const std::string& topic, ros::Time const& time, boost::shared_ptr<T> const& msg,
                boost::shared_ptr<ros::M_string> connection_header)
{
  doWrite(topic, time, *msg, connection_header);
}

template <class T>
void Bag::doWrite(const std::string& topic, ros::Time const& time, T const& msg,
                  boost::shared_ptr<ros::M_string> const& connection_header)
{
  if (mode_ != BagMode::Write)
  {
    throw InvalidModeException{ "Bag must be opened in write mode to support writing." };
  }

  if (time < ros::TIME_MIN)
  {
    throw Exception("Tried to insert a message with time less than ros::TIME_MIN");
  }

  // Whenever we write we increment our revision
  bag_revision_++;

  // Get ID for connection header
  rosbag::ConnectionInfo* connection_info = nullptr;
  uint32_t conn_id = 0;
  if (!connection_header)
  {
    // No connection header: we'll manufacture one, and store by topic

    std::map<std::string, uint32_t>::iterator topic_connection_ids_iter = topic_connection_ids_.find(topic);
    if (topic_connection_ids_iter == topic_connection_ids_.end())
    {
      conn_id = rosbaz::io::narrow<uint32_t>(connections_.size());
      topic_connection_ids_[topic] = conn_id;
    }
    else
    {
      conn_id = topic_connection_ids_iter->second;
      connection_info = connections_[conn_id].get();
    }
  }
  else
  {
    // Store the connection info by the address of the connection header

    // Add the topic name to the connection header, so that when we later search by
    // connection header, we can disambiguate connections that differ only by topic name (i.e.,
    // same callerid, same message type), #3755.  This modified connection header is only used
    // for our bookkeeping, and will not appear in the resulting .bag.
    ros::M_string connection_header_copy(*connection_header);
    connection_header_copy["topic"] = topic;

    std::map<ros::M_string, uint32_t>::iterator header_connection_ids_iter =
        header_connection_ids_.find(connection_header_copy);
    if (header_connection_ids_iter == header_connection_ids_.end())
    {
      conn_id = rosbaz::io::narrow<uint32_t>(connections_.size());
      header_connection_ids_[connection_header_copy] = conn_id;
    }
    else
    {
      conn_id = header_connection_ids_iter->second;
      connection_info = connections_[conn_id].get();
    }
  }

  {
    // Write the chunk header if we're starting a new chunk
    if (!current_block_)
    {
      startWritingChunk(time);
    }

    // Write connection info record, if necessary
    if (connection_info == nullptr)
    {
      rosbag::ConnectionInfo info{};
      info.id = conn_id;
      info.topic = topic;
      info.datatype = std::string(ros::message_traits::datatype(msg));
      info.md5sum = std::string(ros::message_traits::md5sum(msg));
      info.msg_def = std::string(ros::message_traits::definition(msg));
      if (connection_header != NULL)
      {
        info.header = connection_header;
      }
      else
      {
        info.header = boost::make_shared<ros::M_string>();
        (*info.header)["type"] = info.datatype;
        (*info.header)["md5sum"] = info.md5sum;
        (*info.header)["message_definition"] = info.msg_def;
      }
      connections_[conn_id] = std::make_unique<rosbag::ConnectionInfo>(info);
      connection_info = connections_[conn_id].get();
      writeConnectionRecord(connection_info);
    }

    // Add to topic indexes
    rosbag::IndexEntry index_entry;
    index_entry.time = time;
    index_entry.chunk_pos = curr_chunk_info_.pos;
    index_entry.offset = getChunkOffset();

    std::multiset<rosbag::IndexEntry>& chunk_connection_index = curr_chunk_connection_indexes_[connection_info->id];
    chunk_connection_index.insert(chunk_connection_index.end(), index_entry);

    if (mode_ != BagMode::Write)
    {
      std::multiset<rosbag::IndexEntry>& connection_index = connection_indexes_[connection_info->id];
      connection_index.insert(connection_index.end(), index_entry);
    }

    // Increment the connection count
    curr_chunk_info_.connection_counts[connection_info->id]++;

    // Write the message data
    writeMessageDataRecord(conn_id, time, msg);

    // Check if we want to stop this chunk
    uint32_t chunk_size = getChunkOffset();
    ROS_DEBUG("  curr_chunk_size=%d (threshold=%d)", chunk_size, chunk_threshold_);
    if (chunk_size > chunk_threshold_)
    {
      // Empty the outgoing chunk
      stopWritingChunk();

      // We no longer have a valid curr_chunk_info
      curr_chunk_info_.pos = std::numeric_limits<uint64_t>::max();
    }
  }
}

template <class T>
void Bag::writeMessageDataRecord(uint32_t conn_id, ros::Time const& time, T const& msg)
{
  ros::M_string header;
  header[rosbag::OP_FIELD_NAME] = bag_writing::toHeaderString(&rosbag::OP_MSG_DATA);
  header[rosbag::CONNECTION_FIELD_NAME] = bag_writing::toHeaderString(&conn_id);
  header[rosbag::TIME_FIELD_NAME] = bag_writing::toHeaderString(&time);

  // Assemble message in memory first, because we need to write its length
  uint32_t msg_ser_len = ros::serialization::serializationLength(msg);

  record_buffer_.resize(msg_ser_len);

  ros::serialization::OStream s(record_buffer_.data(), msg_ser_len);

  // todo: serialize into the outgoing_chunk_buffer & remove record_buffer_
  ros::serialization::serialize(s, msg);

  ROS_DEBUG("Writing MSG_DATA [%llu:%d]: conn=%d sec=%d nsec=%d data_len=%d",
            static_cast<unsigned long long>(current_block_->block_offset() + current_block_->size()), getChunkOffset(),
            conn_id, time.sec, time.nsec, msg_ser_len);

  bag_writing::writeHeader(*current_block_, header);
  bag_writing::writeDataLength(*current_block_, msg_ser_len);
  current_block_->write(record_buffer_.data(), msg_ser_len);

  // Update the current chunk time range
  if (time > curr_chunk_info_.end_time)
  {
    curr_chunk_info_.end_time = time;
  }
  else if (time < curr_chunk_info_.start_time)
  {
    curr_chunk_info_.start_time = time;
  }
}

}  // namespace rosbaz
