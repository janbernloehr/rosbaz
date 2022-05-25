#include "rosbaz/bag.h"

#include <future>

#include <ros/console.h>
#include <ros/header.h>
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
Bag::Bag(std::shared_ptr<rosbaz::io::IReader> reader) : mode_{ BagMode::Read }, reader_(reader)
{
  assert(reader != nullptr);
}

Bag::Bag(std::shared_ptr<rosbaz::io::IWriter> writer) : mode_{ BagMode::Write }, writer_(writer)
{
  assert(writer != nullptr);
}

Bag::~Bag()
{
  close();
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

  const size_t remainder_size = bag.file_size_ - bag.index_data_pos_;

  const auto bag_tail = reader->read(bag.index_data_pos_, remainder_size);
  bag.parseFileTail(bag_tail);

  if (read_chunk_indices)
  {
    bag.parseChunkIndices(*reader);

    ROS_DEBUG_STREAM("Read indices");
  }

  return bag;
}

Bag Bag::write(std::shared_ptr<rosbaz::io::IWriter> writer)
{
  Bag bag{ writer };

  bag.header_block_ = writer->create_block();

  bag.writeVersion(*bag.header_block_);
  bag.file_header_pos_ = bag.header_block_->size();
  bag.writeFileHeaderRecord(*bag.header_block_);

  bag.header_block_->stage();

  return bag;
}

void Bag::parseFileHeaderRecord(const rosbaz::bag_parsing::Record& file_header_record)
{
  if (mode_ != BagMode::Read)
  {
    throw InvalidModeException{ "Bag must be opened in read mode to support parseFileHeaderRecord." };
  }

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
  if (mode_ != BagMode::Read)
  {
    throw InvalidModeException{ "Bag must be opened in read mode to support parseFileTail." };
  }

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
        connections_[info.id] = std::make_unique<rosbag::ConnectionInfo>(info);
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
  if (mode_ != BagMode::Read)
  {
    throw InvalidModeException{ "Bag must be opened in read mode to support parseIndexSection." };
  }

  // There may be multiple records in the given data span
  uint32_t offset = 0;
  size_t idx = 0;

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

      ROS_DEBUG_STREAM("Found IndexEntry: chunk_pos: " << chunk_ext.chunk_info.pos << " conn: " << connection_id
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

  for (size_t i = 0; i < offsets.size() - 1; ++i)
  {
    record_sizes.emplace(offsets[i], rosbaz::bag_parsing::MessageRecordInfo{ offsets[i], offsets[i + 1] - offsets[i] });
  }

  record_sizes.emplace(offsets.back(), rosbaz::bag_parsing::MessageRecordInfo{
                                           offsets.back(), static_cast<uint32_t>(index_offset - offsets.back() -
                                                                                 chunk_ext.chunk_info.pos) });
}

void Bag::parseChunkInfo(rosbaz::io::IReader& reader, rosbaz::bag_parsing::ChunkExt& chunk_ext,
                         const uint64_t next_chunk_pos)
{
  if (mode_ != BagMode::Read)
  {
    throw InvalidModeException{ "Bag must be opened in read mode to support parseChunkInfo." };
  }
  const auto chunk_header_buffer_and_size = reader.read_header_buffer_and_size(chunk_ext.chunk_info.pos);
  const auto chunk_header_raw = rosbaz::bag_parsing::Header::parse(chunk_header_buffer_and_size.header_buffer);

  chunk_ext.chunk_header = as_chunk_header(chunk_header_raw, chunk_header_buffer_and_size.data_size);

  if (chunk_ext.chunk_header.compression != rosbag::COMPRESSION_NONE)
  {
    std::stringstream msg;
    msg << "Unsupported compression '" << chunk_ext.chunk_header.compression << "'. Only compression '"
        << rosbag::COMPRESSION_NONE << "' is supported.";
    throw rosbaz::UnsupportedRosBagException(msg.str());
  }

  chunk_ext.data_offset =
      chunk_ext.chunk_info.pos + sizeof(uint32_t) + chunk_header_buffer_and_size.header_size + sizeof(uint32_t);
  const std::uint64_t index_offset = chunk_ext.data_offset + chunk_header_buffer_and_size.data_size;
  const std::uint32_t index_size = static_cast<uint32_t>(next_chunk_pos - index_offset);

  ROS_DEBUG_STREAM("Parsed ChunkInfo: chunk pos: " << chunk_ext.chunk_info.pos << " data pos: " << chunk_ext.data_offset
                                                   << " index pos: " << index_offset << " index size: " << index_size);

  const auto index_buffer = reader.read(index_offset, index_size);

  parseIndexSection(chunk_ext, index_buffer, index_offset);
}

void Bag::parseChunkIndices(rosbaz::io::IReader& reader)
{
  if (mode_ != BagMode::Read)
  {
    throw InvalidModeException{ "Bag must be opened in read mode to support parseChunkIndices." };
  }

  // keep the position of the next chunk to determine the size of the index
  // record section.
  std::vector<uint64_t> index_end;
  for (size_t i = 1; i < chunks_.size(); ++i)
  {
    index_end.push_back(chunks_[i].pos);
  }
  index_end.push_back(index_data_pos_);

  // We need to make sure that all inserts are in-place and do not need to re-allocate
  chunk_exts_.reserve(chunks_.size());
  chunk_exts_lookup_.reserve(chunks_.size());

  for (const auto& chunk_info : chunks_)
  {
    chunk_exts_.emplace_back(chunk_info);
    chunk_exts_lookup_[chunk_info.pos] = &chunk_exts_.back();
  }

  {
    std::mutex sync;
    std::vector<std::function<void(void)>> work;

    for (size_t i = 0; i < chunks_.size(); ++i)
    {
      work.emplace_back([this, &reader, &index_end, i]() {
        auto& chunk_ext = chunk_exts_[i];
        const uint64_t next_chunk_pos_value = index_end[i];

        parseChunkInfo(reader, chunk_ext, next_chunk_pos_value);
      });
    }

    io::process_async(sync, work);
  }

  // we add the index entries of every chunk to connection_indexes_ in the order the appear in the
  // bag file
  connection_indexes_.reserve(chunks_.size());
  for (const auto& chunk_ext : chunk_exts_)
  {
    for (const auto& index_entry_ext : chunk_ext.index_entries)
    {
      auto& connection_index = connection_indexes_[index_entry_ext.connection_id];
      connection_index.insert(connection_index.end(), index_entry_ext.index_entry);
    }
  }

  // We build a vector of all message record offsets within the bag file which
  // allows cache strategies to extend all reads to the entire message record
  // to reduce additional round trips.

  std::set<uint64_t> cache_hints;
  cache_hints.emplace(0);
  for (const auto& chunk_ext : chunk_exts_)
  {
    cache_hints.emplace(chunk_ext.data_offset);

    for (const auto& message_record_entry : chunk_ext.message_records)
    {
      const auto& message_record = message_record_entry.second;

      const uint64_t begin = chunk_ext.data_offset + message_record.offset;
      cache_hints.emplace(begin);

      const uint64_t end = begin + message_record.data_size;
      cache_hints.emplace(end);
    }
  }
  cache_hints.emplace(file_size_);

  std::vector<uint64_t> cache_hints_v(cache_hints.begin(), cache_hints.end());

  reader_->set_cache_hints(cache_hints_v);

  chunk_indices_parsed_ = true;
}

std::vector<const rosbag::ConnectionInfo*> Bag::getConnections() const
{
  if (mode_ != BagMode::Read)
  {
    throw InvalidModeException{ "Bag must be opened in read mode." };
  }

  std::vector<const rosbag::ConnectionInfo*> result;

  for (const auto& connection_info : connections_)
  {
    result.push_back(connection_info.second.get());
  }

  return result;
}

ros::Time Bag::getBeginTime() const
{
  if (mode_ != BagMode::Read)
  {
    throw InvalidModeException{ "Bag must be opened in read mode." };
  }
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
  if (mode_ != BagMode::Read)
  {
    throw InvalidModeException{ "Bag must be opened in read mode." };
  }
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
  if (mode_ != BagMode::Read)
  {
    throw InvalidModeException{ "Bag must be opened in read mode." };
  }
  return file_size_;
}

std::uint32_t Bag::getChunkCount() const
{
  if (mode_ != BagMode::Read)
  {
    throw InvalidModeException{ "Bag must be opened in read mode." };
  }
  return chunk_count_;
}

uint32_t Bag::getChunkThreshold() const
{
  return chunk_threshold_;
}

void Bag::setChunkThreshold(const uint32_t chunk_threshold)
{
  chunk_threshold_ = chunk_threshold;
}

CompressionType Bag::getCompression() const
{
  return CompressionType::Uncompressed;
}

std::string Bag::getFilePath() const
{
  if (reader_)
  {
    return reader_->filepath();
  }
  else if (writer_)
  {
    return writer_->filepath();
  }
  else
  {
    throw InvalidModeException("getFilePath requires read or write mode");
  }
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
  return mode_;
}

void Bag::writeVersion(rosbaz::io::Block& block)
{
  const std::string version = std::string("#ROSBAG V") + rosbag::VERSION + std::string("\n");

  ROS_DEBUG("Writing VERSION [%llu]: %s", static_cast<unsigned long long>(block.block_offset() + block.size()),
            version.c_str());

  block.write(version);
}

void Bag::writeFileHeaderRecord(rosbaz::io::Block& block)
{
  connection_count_ = rosbaz::io::narrow<uint32_t>(connections_.size());
  chunk_count_ = rosbaz::io::narrow<uint32_t>(chunks_.size());

  ROS_DEBUG("Writing FILE_HEADER [%llu]: index_pos=%llu connection_count=%d chunk_count=%d",
            static_cast<unsigned long long>(block.block_offset() + block.size()),
            static_cast<unsigned long long>(index_data_pos_), connection_count_, chunk_count_);

  // Write file header record
  ros::M_string header;
  header[rosbag::OP_FIELD_NAME] = rosbaz::bag_writing::toHeaderString(&rosbag::OP_FILE_HEADER);
  header[rosbag::INDEX_POS_FIELD_NAME] = rosbaz::bag_writing::toHeaderString(&index_data_pos_);
  header[rosbag::CONNECTION_COUNT_FIELD_NAME] = rosbaz::bag_writing::toHeaderString(&connection_count_);
  header[rosbag::CHUNK_COUNT_FIELD_NAME] = rosbaz::bag_writing::toHeaderString(&chunk_count_);

  boost::shared_array<uint8_t> header_buffer;
  uint32_t header_len;
  ros::Header::write(header, header_buffer, header_len);
  uint32_t data_len = 0;
  if (header_len < rosbag::FILE_HEADER_LENGTH)
    data_len = rosbag::FILE_HEADER_LENGTH - header_len;
  block.write(reinterpret_cast<const rosbaz::io::byte*>(&header_len), 4);
  block.write(reinterpret_cast<const rosbaz::io::byte*>(header_buffer.get()), header_len);
  block.write(reinterpret_cast<const rosbaz::io::byte*>(&data_len), 4);

  // Pad the file header record out
  if (data_len > 0)
  {
    std::string padding;
    padding.resize(data_len, ' ');
    block.write(padding);
  }
}

void Bag::startWritingChunk(ros::Time time)
{
  current_block_ = writer_->create_block();
  // Initialize chunk info
  curr_chunk_info_.pos = current_block_->block_offset();
  curr_chunk_info_.start_time = time;
  curr_chunk_info_.end_time = time;

  // Write the chunk header, with a place-holder for the data sizes (we'll fill in when the chunk is finished)
  writeChunkHeader(getCompression(), 0, 0);

  // Record where the data section of this chunk started
  curr_chunk_data_pos_ = current_block_->size();
}

void Bag::stopWritingChunk()
{
  // Add this chunk to the index
  chunks_.push_back(curr_chunk_info_);

  // Get the uncompressed and compressed sizes
  const uint32_t uncompressed_size = getChunkOffset();
  const uint32_t compressed_size = uncompressed_size;

  // Write out the indexes and clear them
  writeIndexRecords();

  writeChunkHeader(getCompression(), compressed_size, uncompressed_size);

  curr_chunk_connection_indexes_.clear();

  // Clear the connection counts
  curr_chunk_info_.connection_counts.clear();

  current_block_->stage();
  current_block_.reset();
}

void Bag::writeChunkHeader(CompressionType compression, uint32_t compressed_size, uint32_t uncompressed_size)
{
  rosbag::ChunkHeader chunk_header;
  switch (compression)
  {
    case compression::Uncompressed:
      chunk_header.compression = rosbag::COMPRESSION_NONE;
      break;
    default:
      throw UnsupportedRosBagException("Compression is not supported");
  }
  chunk_header.compressed_size = compressed_size;
  chunk_header.uncompressed_size = uncompressed_size;

  ROS_DEBUG("Writing CHUNK [%llu]: compression=%s compressed=%d uncompressed=%d",
            static_cast<unsigned long long>(current_block_->block_offset() + current_block_->size()),
            chunk_header.compression.c_str(), chunk_header.compressed_size, chunk_header.uncompressed_size);

  ros::M_string header;
  header[rosbag::OP_FIELD_NAME] = rosbaz::bag_writing::toHeaderString(&rosbag::OP_CHUNK);
  header[rosbag::COMPRESSION_FIELD_NAME] = chunk_header.compression;
  header[rosbag::SIZE_FIELD_NAME] = rosbaz::bag_writing::toHeaderString(&chunk_header.uncompressed_size);
  const uint32_t header_size = writeHeader(header, 0);

  writeDataLength(chunk_header.compressed_size, header_size);
}

uint32_t Bag::writeHeader(ros::M_string const& fields, boost::optional<size_t> offset)
{
  boost::shared_array<uint8_t> header_buffer;
  uint32_t header_len;
  ros::Header::write(fields, header_buffer, header_len);
  current_block_->write(reinterpret_cast<const rosbaz::io::byte*>(&header_len), 4, offset);
  current_block_->write(reinterpret_cast<const rosbaz::io::byte*>(header_buffer.get()), header_len,
                        offset ? boost::optional<size_t>{ *offset + 4 } : boost::none);

  return 4 + header_len;
}

uint32_t Bag::writeDataLength(uint32_t data_len, boost::optional<size_t> offset)
{
  current_block_->write(reinterpret_cast<const rosbaz::io::byte*>(&data_len), 4, offset);
  return 4;
}

void Bag::writeIndexRecords()
{
  for (const auto& chunk_connection_index : curr_chunk_connection_indexes_)
  {
    uint32_t connection_id = chunk_connection_index.first;
    const auto& index = chunk_connection_index.second;

    // Write the index record header
    const uint32_t index_size = rosbaz::io::narrow<uint32_t>(index.size());
    ros::M_string header;
    header[rosbag::OP_FIELD_NAME] = rosbaz::bag_writing::toHeaderString(&rosbag::OP_INDEX_DATA);
    header[rosbag::CONNECTION_FIELD_NAME] = rosbaz::bag_writing::toHeaderString(&connection_id);
    header[rosbag::VER_FIELD_NAME] = rosbaz::bag_writing::toHeaderString(&rosbag::INDEX_VERSION);
    header[rosbag::COUNT_FIELD_NAME] = rosbaz::bag_writing::toHeaderString(&index_size);
    writeHeader(header);

    writeDataLength(index_size * 12);

    ROS_DEBUG("Writing INDEX_DATA: connection=%d ver=%d count=%d", connection_id, rosbag::INDEX_VERSION, index_size);

    // Write the index record data (pairs of timestamp and position in file)
    for (const auto& e : index)
    {
      current_block_->write(reinterpret_cast<const rosbaz::io::byte*>(&e.time.sec), 4);
      current_block_->write(reinterpret_cast<const rosbaz::io::byte*>(&e.time.nsec), 4);
      current_block_->write(reinterpret_cast<const rosbaz::io::byte*>(&e.offset), 4);

      ROS_DEBUG("  - %d.%d: %d", e.time.sec, e.time.nsec, e.offset);
    }
  }
}

uint32_t Bag::getChunkOffset() const
{
  return rosbaz::io::narrow<uint32_t>(current_block_->size() - curr_chunk_data_pos_);
}

void Bag::writeConnectionRecords()
{
  for (const auto& connection_info : connections_)
  {
    writeConnectionRecord(connection_info.second.get());
  }
}

void Bag::writeConnectionRecord(rosbag::ConnectionInfo const* connection_info)
{
  ROS_DEBUG("Writing CONNECTION [%llu:%d]: topic=%s id=%d",
            static_cast<unsigned long long>(current_block_->block_offset() + current_block_->size()), getChunkOffset(),
            connection_info->topic.c_str(), connection_info->id);

  ros::M_string header;
  header[rosbag::OP_FIELD_NAME] = rosbaz::bag_writing::toHeaderString(&rosbag::OP_CONNECTION);
  header[rosbag::TOPIC_FIELD_NAME] = connection_info->topic;
  header[rosbag::CONNECTION_FIELD_NAME] = rosbaz::bag_writing::toHeaderString(&connection_info->id);

  writeHeader(header);
  writeHeader(*connection_info->header);
}

void Bag::close()
{
  if (reader_)
  {
    return;
  }

  if (writer_)
  {
    closeWrite();
    writer_.reset();
  }
}

void Bag::closeWrite()
{
  stopWriting();
}

void Bag::stopWriting()
{
  if (current_block_)
  {
    stopWritingChunk();
  }

  current_block_ = writer_->create_block();

  index_data_pos_ = current_block_->block_offset();
  writeConnectionRecords();
  writeChunkInfoRecords();
  current_block_->stage();

  current_block_ = writer_->replace_block(*header_block_);

  writeVersion(*current_block_);
  writeFileHeaderRecord(*current_block_);

  current_block_->stage();

  writer_->commit_blocks();
}

void Bag::writeChunkInfoRecords()
{
  for (const auto& chunk_info : chunks_)
  {
    // Write the chunk info header
    ros::M_string header;
    const uint32_t chunk_connection_count = rosbaz::io::narrow<uint32_t>(chunk_info.connection_counts.size());
    header[rosbag::OP_FIELD_NAME] = rosbaz::bag_writing::toHeaderString(&rosbag::OP_CHUNK_INFO);
    header[rosbag::VER_FIELD_NAME] = rosbaz::bag_writing::toHeaderString(&rosbag::CHUNK_INFO_VERSION);
    header[rosbag::CHUNK_POS_FIELD_NAME] = rosbaz::bag_writing::toHeaderString(&chunk_info.pos);
    header[rosbag::START_TIME_FIELD_NAME] = rosbaz::bag_writing::toHeaderString(&chunk_info.start_time);
    header[rosbag::END_TIME_FIELD_NAME] = rosbaz::bag_writing::toHeaderString(&chunk_info.end_time);
    header[rosbag::COUNT_FIELD_NAME] = rosbaz::bag_writing::toHeaderString(&chunk_connection_count);

    ROS_DEBUG("Writing CHUNK_INFO [%llu]: ver=%d pos=%llu start=%d.%d end=%d.%d",
              static_cast<unsigned long long>(current_block_->block_offset() + current_block_->size()),
              rosbag::CHUNK_INFO_VERSION, static_cast<unsigned long long>(chunk_info.pos), chunk_info.start_time.sec,
              chunk_info.start_time.nsec, chunk_info.end_time.sec, chunk_info.end_time.nsec);

    writeHeader(header);

    writeDataLength(8 * chunk_connection_count);

    // Write the topic names and counts
    for (const auto& connection_count : chunk_info.connection_counts)
    {
      const uint32_t connection_id = connection_count.first;
      const uint32_t count = connection_count.second;

      current_block_->write(reinterpret_cast<const rosbaz::io::byte*>(&connection_id), 4);
      current_block_->write(reinterpret_cast<const rosbaz::io::byte*>(&count), 4);

      ROS_DEBUG("  - %d: %d", connection_id, count);
    }
  }
}

}  // namespace rosbaz
