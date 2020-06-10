#include "rosbaz/bag.h"

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

namespace rosbaz {

Bag::Bag(std::shared_ptr<rosbaz::io::IReader> reader) : reader_(reader) {
  assert(reader != nullptr);
}

Bag Bag::read(std::shared_ptr<rosbaz::io::IReader> reader,
              bool read_chunk_indices) {
  Bag bag{reader};

  constexpr size_t kVersionHeaderSize = 13;
  constexpr size_t kFileHeaderTotalSize =
      2 * sizeof(uint32_t) + rosbag::FILE_HEADER_LENGTH;

  const auto version_and_file_header_buffer =
      reader->read(0, kVersionHeaderSize + kFileHeaderTotalSize);
  const rosbaz::DataSpan version_and_file_header_span{
      version_and_file_header_buffer};

  if (!rosbaz::bag_parsing::verifyVersionHeader(
          version_and_file_header_span.subspan(0, kVersionHeaderSize))) {
    std::stringstream msg;
    msg << "Only rosbag version 2.0 is supported";
    throw rosbaz::UnsupportedRosBagException(msg.str());
  }

  bag.file_header_pos_ = kVersionHeaderSize;

  const auto file_header_record = rosbaz::bag_parsing::Record::parse(
      version_and_file_header_span.subspan(kVersionHeaderSize));

  bag.parseFileHeaderRecord(file_header_record);
  bag.file_size_ = reader->size();

  ROS_DEBUG_STREAM("chunk_count: " << bag.chunk_count_);
  ROS_DEBUG_STREAM("conn_count: " << bag.connection_count_);
  ROS_DEBUG_STREAM("index_pos: " << bag.index_data_pos_);

  ROS_DEBUG_STREAM("Read file header");

  std::int32_t reminder_size =
      static_cast<std::int32_t>(bag.file_size_ - bag.index_data_pos_);

  const auto bag_tail = reader->read(bag.index_data_pos_, reminder_size);
  bag.parseFileTail(bag_tail);

  if (read_chunk_indices) {
    bag.parseChunkIndices(*reader);

    ROS_DEBUG_STREAM("Read indices");
  }

  return bag;
}

void Bag::parseFileHeaderRecord(
    const rosbaz::bag_parsing::Record &file_header_record) {
  auto h = rosbaz::bag_parsing::Header::parse(file_header_record.header);

  if (h.op != rosbag::OP_FILE_HEADER) {
    std::stringstream msg;
    msg << "Expected op " << static_cast<int>(rosbag::OP_FILE_HEADER)
        << " in file header but found " << static_cast<int>(h.op);
    throw rosbaz::UnsupportedRosBagException(msg.str());
  }

  if (h.fields.count(rosbag::ENCRYPTOR_FIELD_NAME) != 0) {
    std::stringstream msg;
    msg << "This bag uses an encryptor which is not supported";
    throw rosbaz::UnsupportedRosBagException(msg.str());
  }

  chunk_count_ = rosbaz::io::read_little_endian<uint32_t>(
      h.fields.at(rosbag::CHUNK_COUNT_FIELD_NAME));
  connection_count_ = rosbaz::io::read_little_endian<uint32_t>(
      h.fields[rosbag::CONNECTION_COUNT_FIELD_NAME]);
  index_data_pos_ = rosbaz::io::read_little_endian<uint64_t>(
      h.fields[rosbag::INDEX_POS_FIELD_NAME]);
}

void Bag::parseFileTail(rosbaz::DataSpan bag_tail) {
  std::uint64_t offset = 0;

  while (offset < bag_tail.size()) {
    auto record = rosbaz::bag_parsing::Record::parse(bag_tail.subspan(offset));
    auto header = rosbaz::bag_parsing::Header::parse(record.header);

    ROS_DEBUG_STREAM("op: " << static_cast<int>(header.op));

    switch (header.op) {
    case rosbag::OP_CONNECTION: {
      rosbag::ConnectionInfo info = as_connection_info(header, record.data);
      connections_[info.id] = info;
    } break;
    case rosbag::OP_CHUNK_INFO: {
      chunk_infos_.push_back(as_chunk_info(header, record.data));
    } break;
    default:
      ROS_WARN_STREAM("Not implemented op=" << static_cast<int>(header.op));
      break;
    }

    offset += record.total_size();
  }
}

void Bag::parseIndexSection(rosbaz::bag_parsing::ChunkExt &chunk_ext,
                            rosbaz::DataSpan chunk_index,
                            const uint64_t index_offset) {
  // There may be multiple records in the given data span
  uint32_t offset = 0;

  std::vector<uint32_t> offsets;

  while (offset < chunk_index.size()) {
    const auto index_record =
        rosbaz::bag_parsing::Record::parse(chunk_index.subspan(offset));
    const auto index_header =
        rosbaz::bag_parsing::Header::parse(index_record.header);

    if (index_header.op != rosbag::OP_INDEX_DATA) {
      std::stringstream msg;
      msg << "Unexpected op code " << static_cast<int>(index_header.op)
          << " while parsing index section. Only op code "
          << static_cast<int>(rosbag::OP_INDEX_DATA) << " is supported.";
      throw rosbaz::UnsupportedRosBagException(msg.str());
    }

    const uint32_t index_version = rosbaz::io::read_little_endian<uint32_t>(
        index_header.fields.at(rosbag::VER_FIELD_NAME));

    if (index_version != rosbag::INDEX_VERSION) {
      std::stringstream msg;
      msg << "Unexpected index version " << index_version << ". Only version "
          << rosbag::INDEX_VERSION << " is supported.";
      throw rosbaz::UnsupportedRosBagException(msg.str());
    }

    const uint32_t connection_id = rosbaz::io::read_little_endian<uint32_t>(
        index_header.fields.at(rosbag::CONNECTION_FIELD_NAME));
    const uint32_t count = rosbaz::io::read_little_endian<uint32_t>(
        index_header.fields.at(rosbag::COUNT_FIELD_NAME));

    auto &connection_index = connection_indexes_[connection_id];

    for (uint32_t i = 0; i < count; i++) {
      rosbag::IndexEntry index_entry;

      index_entry.time = rosbaz::bag_parsing::unpack_time(
          rosbaz::io::read_little_endian<uint64_t>(index_record.data, i * 12));

      index_entry.offset = rosbaz::io::read_little_endian<uint32_t>(
          index_record.data, i * 12 + 8);
      index_entry.chunk_pos = chunk_ext.chunk_info.pos;

      ROS_DEBUG_STREAM("chunk_pos: " << chunk_ext.chunk_info.pos
                                     << " conn: " << connection_id
                                     << " offset: " << index_entry.offset);

      connection_index.insert(connection_index.end(), index_entry);
      offsets.push_back(index_entry.offset);
    }

    offset += index_record.total_size();
  }

  std::sort(offsets.begin(), offsets.end());

  auto &record_sizes = chunk_ext.message_records;

  for (size_t i = 0; i < offsets.size() - 1; ++i) {
    record_sizes.emplace_back(rosbaz::bag_parsing::MessageRecordInfo{
        offsets[i], static_cast<uint32_t>(offsets[i + 1] - offsets[i])});
  }

  record_sizes.emplace_back(rosbaz::bag_parsing::MessageRecordInfo{
      offsets.back(), static_cast<uint32_t>(index_offset - offsets.back() -
                                            chunk_ext.chunk_info.pos)});
}

void Bag::parseChunkIndices(rosbaz::io::IReader &reader) {
  // keep the position of the next chunk to determine the size of the index
  // record section.
  std::vector<uint64_t> index_end;
  for (size_t i = 1; i < chunk_infos_.size(); ++i) {
    index_end.push_back(chunk_infos_[i].pos);
  }
  index_end.push_back(index_data_pos_);

  auto next_chunk_pos = index_end.begin();

  for (const auto &chunk_info : chunk_infos_) {
    const auto chunk_header_buffer_and_size =
        reader.read_header_buffer_and_size(chunk_info.pos);
    const auto chunk_header_raw = rosbaz::bag_parsing::Header::parse(
        chunk_header_buffer_and_size.header_buffer);

    const auto chunk_header = as_chunk_header(
        chunk_header_raw, chunk_header_buffer_and_size.data_size);

    if (chunk_header.compression != rosbag::COMPRESSION_NONE) {
      std::stringstream msg;
      msg << "Unsupported compression '" << chunk_header.compression
          << "'. Only compression '" << rosbag::COMPRESSION_NONE
          << "' is supported.";
      throw rosbaz::UnsupportedRosBagException(msg.str());
    }

    const uint64_t data_offset = chunk_info.pos + sizeof(uint32_t) +
                                 chunk_header_buffer_and_size.header_size +
                                 sizeof(uint32_t);

    const uint64_t index_offset =
        data_offset + chunk_header_buffer_and_size.data_size;

    chunks_.emplace_back(chunk_info, chunk_header, data_offset,
                         chunk_header_buffer_and_size.data_size, index_offset,
                         static_cast<uint32_t>(*next_chunk_pos - index_offset));
    auto &chunk_ext = chunks_.back();

    ROS_DEBUG_STREAM("chunk pos: " << chunk_ext.chunk_info.pos
                                   << " data pos: " << chunk_ext.data_offset
                                   << " index pos: " << chunk_ext.index_offset
                                   << " index size: " << chunk_ext.index_size);

    const auto index_buffer =
        reader.read(chunk_ext.index_offset, chunk_ext.index_size);

    parseIndexSection(chunk_ext, index_buffer, index_offset);

    ++next_chunk_pos;
  }

  chunk_indices_parsed_ = true;
}

std::vector<const rosbag::ConnectionInfo *> Bag::getConnections() const {
  std::vector<const rosbag::ConnectionInfo *> result;

  for (const auto &connection_info : connections_) {
    result.push_back(&connection_info.second);
  }

  return result;
}

std::uint32_t Bag::getMessageCountForConnectionId(uint32_t id) const {
  std::uint32_t count = 0;

  for (const auto &chunk_info : chunk_infos_) {
    if (chunk_info.connection_counts.count(id) != 0) {
      count += chunk_info.connection_counts.at(id);
    }
  }

  return count;
}

ros::Time Bag::getBeginTime() const {
  ros::Time begin = ros::TIME_MAX;

  for (const auto &chunk_info : chunk_infos_) {
    if (chunk_info.start_time < begin) {
      begin = chunk_info.start_time;
    }
  }

  return begin;
}

ros::Time Bag::getEndTime() const {
  ros::Time end = ros::TIME_MIN;

  for (const auto &chunk_info : chunk_infos_) {
    if (chunk_info.end_time > end) {
      end = chunk_info.end_time;
    }
  }

  return end;
}

} // namespace rosbaz