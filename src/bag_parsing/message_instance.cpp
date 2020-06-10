#include "rosbaz/bag_parsing/message_instance.h"


namespace rosbaz {
namespace bag_parsing {

MessageInstance::MessageInstance(const rosbag::ConnectionInfo& connection_info,
                                 const rosbag::IndexEntry& index,
                                 const AzBag& bag,
                                 rosbaz::io::IReader& reader)
    : m_connection_info(&connection_info), m_index_entry(&index), m_bag(&bag), m_reader(&reader) {}

const ros::Time& MessageInstance::getTime() const { return m_index_entry->time; }
const std::string& MessageInstance::getTopic() const { return m_connection_info->topic; }
const std::string& MessageInstance::getDataType() const { return m_connection_info->datatype; }
const std::string& MessageInstance::getMD5Sum() const { return m_connection_info->md5sum; }
const std::string& MessageInstance::getMessageDefinition() const {
  return m_connection_info->msg_def;
}

void MessageInstance::getOffsetAndSize(uint64_t& record_offset, uint32_t& record_size) const {
  assert(m_bag->chunk_indices_parsed_);

  auto found_chunk =
      std::find_if(m_bag->chunks_.begin(), m_bag->chunks_.end(), [this](const ChunkExt& chunk_ext) {
        return chunk_ext.chunk_info.pos == m_index_entry->chunk_pos;
      });

  if (found_chunk == m_bag->chunks_.end()) {
    throw std::runtime_error("Could not find chunk information for chunk.");
  }

  auto found_size = std::find_if(
      found_chunk->message_records.begin(), found_chunk->message_records.end(),
      [this](const MessageRecordInfo& info) { return info.offset == m_index_entry->offset; });

  if (found_size == found_chunk->message_records.end()) {
    throw std::runtime_error("Could not find message size of message.");
  }

  ROS_DEBUG_STREAM("chunk_pos: " << m_index_entry->chunk_pos << " offset: " << found_size->offset
                               << " size: " << found_size->data_size);

  record_offset = found_chunk->data_offset + m_index_entry->offset;
  record_size = found_size->data_size;
}

//! Size of serialized message
uint32_t MessageInstance::size() const {
  uint64_t record_offset;
  uint32_t record_size;

  getOffsetAndSize(record_offset, record_size);

  return record_size;
}
}  // namespace bag_parsing
}  // namespace rosbaz