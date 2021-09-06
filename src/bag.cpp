#include "rosbaz/bag.h"

#include <future>

#include <ros/console.h>
#include <rosbag/constants.h>
#include <rosbag/structures.h>

#include "rosbaz/bag_parsing/conversion.h"
#include "rosbaz/bag_parsing/header.h"
#include "rosbaz/bag_parsing/record.h"
#include "rosbaz/common.h"
#include "rosbaz/exceptions.h"
#include "rosbaz/io/io_helpers.h"
#include "rosbaz/io/reader.h"
#include "rosbaz/io/thread_pool.h"
#include "rosbaz/io/util.h"

namespace rosbaz
{
Bag::Bag(std::shared_ptr<rosbaz::io::IReader> reader) : reader_(reader)
{
  assert(reader != nullptr);
}

Bag Bag::read(std::shared_ptr<rosbaz::io::IReader> reader, bool read_chunk_indices)
{
  Bag bag{ reader };

  constexpr size_t kVersionHeaderSize = 13;
  constexpr size_t kFileHeaderTotalSize = 2 * sizeof(uint32_t) + rosbag::FILE_HEADER_LENGTH;

  const auto version_and_file_header_buffer = reader->read(0, kVersionHeaderSize + kFileHeaderTotalSize);
  const rosbaz::DataSpan version_and_file_header_span{ version_and_file_header_buffer };

  if (!rosbaz::bag_parsing::verifyVersionHeader(version_and_file_header_span.subspan(0, kVersionHeaderSize)))
  {
    std::stringstream msg;
    msg << "Only rosbag version 2.0 is supported";
    throw rosbaz::UnsupportedRosBagException(msg.str());
  }

  bag.file_header_pos_ = kVersionHeaderSize;

  const auto file_header_record =
      rosbaz::bag_parsing::Record::parse(version_and_file_header_span.subspan(kVersionHeaderSize));

  bag.parseFileHeaderRecord(file_header_record);
  bag.file_size_ = reader->size();

  ROS_DEBUG_STREAM("chunk_count: " << bag.chunk_count_);
  ROS_DEBUG_STREAM("conn_count: " << bag.connection_count_);
  ROS_DEBUG_STREAM("index_pos: " << bag.index_data_pos_);

  ROS_DEBUG_STREAM("Read file header");

  if (bag.index_data_pos_ > bag.file_size_)
  {
    std::stringstream msg;
    msg << "Expected index at " << bag.index_data_pos_ << " but bag is only " << bag.file_size_
        << " bytes long. Try reindexing.";
    throw RosBagUnindexedException(msg.str());
  }

  const size_t reminder_size = static_cast<size_t>(bag.file_size_ - bag.index_data_pos_);

  const auto bag_tail = reader->read(bag.index_data_pos_, reminder_size);
  bag.parseFileTail(bag_tail);

  if (read_chunk_indices)
  {
    bag.parseChunkIndices(*reader);

    ROS_DEBUG_STREAM("Read indices");
  }

  return bag;
}

void Bag::parseFileHeaderRecord(const rosbaz::bag_parsing::Record& file_header_record)
{
  auto h = rosbaz::bag_parsing::Header::parse(file_header_record.header);

  if (h.op != rosbag::OP_FILE_HEADER)
  {
    std::stringstream msg;
    msg << "Expected op " << static_cast<int>(rosbag::OP_FILE_HEADER) << " in file header but found "
        << static_cast<int>(h.op);
    throw rosbaz::UnsupportedRosBagException(msg.str());
  }

  if (h.fields.count(rosbag::ENCRYPTOR_FIELD_NAME) != 0)
  {
    std::stringstream msg;
    msg << "This bag uses an encryptor which is not supported";
    throw rosbaz::UnsupportedRosBagException(msg.str());
  }

  chunk_count_ = h.read_field_little_endian<uint32_t>(rosbag::CHUNK_COUNT_FIELD_NAME);
  connection_count_ = h.read_field_little_endian<uint32_t>(rosbag::CONNECTION_COUNT_FIELD_NAME);
  index_data_pos_ = h.read_field_little_endian<uint64_t>(rosbag::INDEX_POS_FIELD_NAME);

  if (index_data_pos_ == 0)
  {
    std::stringstream msg;
    msg << "Index not found in bagfile. Try reindexing.";
    throw RosBagUnindexedException(msg.str());
  }
}

void Bag::parseFileTail(rosbaz::DataSpan bag_tail)
{
  std::uint64_t offset = 0;

  while (offset < bag_tail.size())
  {
    auto record = rosbaz::bag_parsing::Record::parse(bag_tail.subspan(offset));
    auto header = rosbaz::bag_parsing::Header::parse(record.header);

    ROS_DEBUG_STREAM("op: " << static_cast<int>(header.op));

    switch (header.op)
    {
      case rosbag::OP_CONNECTION: {
        rosbag::ConnectionInfo info = as_connection_info(header, record.data);
        connections_[info.id] = info;
      }
      break;
      case rosbag::OP_CHUNK_INFO: {
        chunks_.push_back(as_chunk_info(header, record.data));
      }
      break;
      default:
        ROS_WARN_STREAM("Not implemented op=" << static_cast<int>(header.op));
        break;
    }

    offset += record.total_size();
  }
}

void Bag::parseIndexSection(rosbaz::bag_parsing::ChunkExt& chunk_ext, rosbaz::DataSpan chunk_index,
                            const uint64_t index_offset)
{
  // There may be multiple records in the given data span
  uint32_t offset = 0;
  int idx = 0;

  std::vector<uint32_t> offsets;

  while (idx < chunk_ext.chunk_info.connection_counts.size())
  {
    const auto index_record = rosbaz::bag_parsing::Record::parse(chunk_index.subspan(offset));
    const auto index_header = rosbaz::bag_parsing::Header::parse(index_record.header);

    if (index_header.op != rosbag::OP_INDEX_DATA)
    {
      std::stringstream msg;
      msg << "Unexpected op code " << static_cast<int>(index_header.op) << " while parsing index section. Only op code "
          << static_cast<int>(rosbag::OP_INDEX_DATA) << " is supported.";
      throw rosbaz::UnsupportedRosBagException(msg.str());
    }

    const uint32_t index_version = index_header.read_field_little_endian<uint32_t>(rosbag::VER_FIELD_NAME);

    if (index_version != rosbag::INDEX_VERSION)
    {
      std::stringstream msg;
      msg << "Unexpected index version " << index_version << ". Only version " << rosbag::INDEX_VERSION
          << " is supported.";
      throw rosbaz::UnsupportedRosBagException(msg.str());
    }

    const uint32_t connection_id = index_header.read_field_little_endian<uint32_t>(rosbag::CONNECTION_FIELD_NAME);
    const uint32_t count = index_header.read_field_little_endian<uint32_t>(rosbag::COUNT_FIELD_NAME);

    for (uint32_t i = 0; i < count; i++)
    {
      rosbag::IndexEntry index_entry;

      index_entry.time =
          rosbaz::bag_parsing::unpack_time(rosbaz::io::read_little_endian<uint64_t>(index_record.data, i * 12));

      index_entry.offset = rosbaz::io::read_little_endian<uint32_t>(index_record.data, i * 12 + 8);
      index_entry.chunk_pos = chunk_ext.chunk_info.pos;

      ROS_DEBUG_STREAM("chunk_pos: " << chunk_ext.chunk_info.pos << " conn: " << connection_id
                                     << " offset: " << index_entry.offset);

      // adding index entries to the global connection_indexes_ is delicate since the latter is sorted on every
      // modification using the index entry's time as a key. Moreover, there may be multiple index entries with the
      // same time.
      // Since many chunks are parsed in parallel, we will build a list of index entries here and add them
      // in the order the chunks appear in the bag from a single thread later.
      chunk_ext.index_entries.emplace_back(connection_id, index_entry);
      offsets.push_back(index_entry.offset);
    }

    offset += index_record.total_size();
    ++idx;
  }

  std::sort(offsets.begin(), offsets.end());

  auto& record_sizes = chunk_ext.message_records;
  record_sizes.reserve(offsets.size());

  for (size_t i = 0; i < offsets.size() - 1; ++i)
  {
    record_sizes.emplace(offsets[i], rosbaz::bag_parsing::MessageRecordInfo{
                                         offsets[i], static_cast<uint32_t>(offsets[i + 1] - offsets[i]) });
  }

  record_sizes.emplace(offsets.back(), rosbaz::bag_parsing::MessageRecordInfo{
                                           offsets.back(), static_cast<uint32_t>(index_offset - offsets.back() -
                                                                                 chunk_ext.chunk_info.pos) });
}

void Bag::parseChunkInfo(std::mutex& sync, rosbaz::io::IReader& reader, const rosbag::ChunkInfo& chunk_info,
                         const uint64_t next_chunk_pos)
{
  const auto chunk_header_buffer_and_size = reader.read_header_buffer_and_size(chunk_info.pos);
  const auto chunk_header_raw = rosbaz::bag_parsing::Header::parse(chunk_header_buffer_and_size.header_buffer);

  const auto chunk_header = as_chunk_header(chunk_header_raw, chunk_header_buffer_and_size.data_size);

  if (chunk_header.compression != rosbag::COMPRESSION_NONE)
  {
    std::stringstream msg;
    msg << "Unsupported compression '" << chunk_header.compression << "'. Only compression '"
        << rosbag::COMPRESSION_NONE << "' is supported.";
    throw rosbaz::UnsupportedRosBagException(msg.str());
  }

  const uint64_t data_offset =
      chunk_info.pos + sizeof(uint32_t) + chunk_header_buffer_and_size.header_size + sizeof(uint32_t);

  const uint64_t index_offset = data_offset + chunk_header_buffer_and_size.data_size;

  rosbaz::bag_parsing::ChunkExt* chunk_ext = nullptr;

  {
    std::lock_guard<std::mutex> guard(sync);
    chunk_exts_.emplace_back(chunk_info, chunk_header, data_offset, chunk_header_buffer_and_size.data_size,
                             index_offset, static_cast<uint32_t>(next_chunk_pos - index_offset));

    // this only works since we reserved chunk_exts_ to the correct size so that no reallocation occurs
    chunk_ext = &chunk_exts_.back();
    chunk_exts_lookup_[chunk_info.pos] = chunk_ext;
  }

  ROS_DEBUG_STREAM("chunk pos: " << chunk_ext->chunk_info.pos << " data pos: " << chunk_ext->data_offset
                                 << " index pos: " << chunk_ext->index_offset
                                 << " index size: " << chunk_ext->index_size);

  const auto index_buffer = reader.read(chunk_ext->index_offset, chunk_ext->index_size);

  parseIndexSection(*chunk_ext, index_buffer, index_offset);
}

void Bag::parseChunkIndices(rosbaz::io::IReader& reader)
{
  // We need to make sure that all inserts are in-place and do not need to re-allocate
  chunk_exts_.reserve(chunks_.size());
  chunk_exts_lookup_.reserve(chunks_.size());

  // keep the position of the next chunk to determine the size of the index
  // record section.
  std::vector<uint64_t> index_end;
  for (size_t i = 1; i < chunks_.size(); ++i)
  {
    index_end.push_back(chunks_[i].pos);
  }
  index_end.push_back(index_data_pos_);

  auto next_chunk_pos = index_end.begin();

  {
    std::mutex sync;
    std::vector<std::function<void(void)>> work;

    for (const auto& chunk_info : chunks_)
    {
      const uint64_t next_chunk_pos_value = *next_chunk_pos;
      work.emplace_back([this, &sync, &reader, &chunk_info, next_chunk_pos_value]() {
        parseChunkInfo(sync, reader, chunk_info, next_chunk_pos_value);
      });
      ++next_chunk_pos;
    }

    io::process_async(sync, work);
  }

  // we add the index entries of every chunk to connection_indexes_ in the order the appear in the
  // bag file
  connection_indexes_.reserve(chunks_.size());
  for (const auto& chunk_info : chunks_)
  {
    const auto& found_chunk = chunk_exts_lookup_.at(chunk_info.pos);

    for (const auto& index_entry_ext : found_chunk->index_entries)
    {
      auto& connection_index = connection_indexes_[index_entry_ext.connection_id];
      connection_index.insert(connection_index.end(), index_entry_ext.index_entry);
    }
  }

  chunk_indices_parsed_ = true;
}

std::vector<const rosbag::ConnectionInfo*> Bag::getConnections() const
{
  std::vector<const rosbag::ConnectionInfo*> result;

  for (const auto& connection_info : connections_)
  {
    result.push_back(&connection_info.second);
  }

  return result;
}

std::uint32_t Bag::getMessageCountForConnectionId(uint32_t id) const
{
  std::uint32_t count = 0;

  for (const auto& chunk_info : chunks_)
  {
    if (chunk_info.connection_counts.count(id) != 0)
    {
      count += chunk_info.connection_counts.at(id);
    }
  }

  return count;
}

ros::Time Bag::getBeginTime() const
{
  ros::Time begin = ros::TIME_MAX;

  for (const auto& chunk_info : chunks_)
  {
    if (chunk_info.start_time < begin)
    {
      begin = chunk_info.start_time;
    }
  }

  return begin;
}

ros::Time Bag::getEndTime() const
{
  ros::Time end = ros::TIME_MIN;

  for (const auto& chunk_info : chunks_)
  {
    if (chunk_info.end_time > end)
    {
      end = chunk_info.end_time;
    }
  }

  return end;
}

std::uint64_t Bag::getSize() const
{
  return file_size_;
}

std::uint32_t Bag::getChunkCount() const
{
  return chunk_count_;
}

uint32_t Bag::getChunkThreshold() const
{
  return 0;
}

CompressionType Bag::getCompression() const
{
  return CompressionType::Uncompressed;
}

std::string Bag::getFilePath() const
{
  return reader_->filepath();
}

uint32_t Bag::getMajorVersion() const
{
  return 2;
}

uint32_t Bag::getMinorVersion() const
{
  return 0;
}

BagMode Bag::getMode() const
{
  return BagMode::Read;
}

}  // namespace rosbaz
