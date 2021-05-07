#include "rosbaz/message_instance.h"

#include <chrono>

#include "rosbaz/exceptions.h"

namespace rosbaz
{
MessageInstance::MessageInstance(const rosbag::ConnectionInfo& connection_info, const rosbag::IndexEntry& index,
                                 const Bag& bag)
  : m_connection_info(&connection_info), m_index_entry(&index), m_bag(&bag)
{
}

const ros::Time& MessageInstance::getTime() const
{
  return m_index_entry->time;
}
const std::string& MessageInstance::getTopic() const
{
  return m_connection_info->topic;
}
const std::string& MessageInstance::getDataType() const
{
  return m_connection_info->datatype;
}
const std::string& MessageInstance::getMD5Sum() const
{
  return m_connection_info->md5sum;
}

const std::string& MessageInstance::getMessageDefinition() const
{
  return m_connection_info->msg_def;
}

boost::shared_ptr<ros::M_string> MessageInstance::getConnectionHeader() const
{
  return m_connection_info->header;
}

std::string MessageInstance::getCallerId() const
{
  ros::M_string::const_iterator header_iter = m_connection_info->header->find("callerid");
  return header_iter != m_connection_info->header->end() ? header_iter->second : std::string("");
}

bool MessageInstance::isLatching() const
{
  ros::M_string::const_iterator header_iter = m_connection_info->header->find("latching");
  return header_iter != m_connection_info->header->end() && header_iter->second == "1";
}

void MessageInstance::getOffsetAndSize(uint64_t& record_offset, uint32_t& record_size) const
{
  assert(m_bag->chunk_indices_parsed_);

  auto found_chunk = std::find_if(m_bag->chunk_exts_.begin(), m_bag->chunk_exts_.end(),
                                  [this](const rosbaz::bag_parsing::ChunkExt& chunk_ext) {
                                    return chunk_ext.chunk_info.pos == m_index_entry->chunk_pos;
                                  });

  if (found_chunk == m_bag->chunk_exts_.end())
  {
    std::stringstream msg;
    msg << "Requested chunk at pos=" << m_index_entry->chunk_pos << " could not be found in index.";
    throw rosbaz::InvalidBagIndexException(msg.str());
  }

  auto found_size = std::find_if(
      found_chunk->message_records.begin(), found_chunk->message_records.end(),
      [this](const rosbaz::bag_parsing::MessageRecordInfo& info) { return info.offset == m_index_entry->offset; });

  if (found_size == found_chunk->message_records.end())
  {
    std::stringstream msg;
    msg << "Requested message record with offset=" << m_index_entry->offset << " and chunk pos "
        << m_index_entry->chunk_pos << " could not be found in index.";
    throw rosbaz::InvalidBagIndexException(msg.str());
  }

  ROS_DEBUG_STREAM("chunk_pos: " << m_index_entry->chunk_pos << " offset: " << found_size->offset
                                 << " size: " << found_size->data_size);

  record_offset = found_chunk->data_offset + m_index_entry->offset;
  record_size = found_size->data_size;
}

std::vector<rosbaz::io::byte> MessageInstance::read_subset(uint32_t offset, uint32_t size) const
{
  uint64_t record_offset;
  uint32_t record_size;

  getOffsetAndSize(record_offset, record_size);

  if (!m_header_buffer_and_size)
  {
    m_header_buffer_and_size = m_bag->reader_->read_header_buffer_and_size(record_offset);
  }

  if (offset + size > m_header_buffer_and_size->data_size)
  {
    std::stringstream msg;
    msg << "Requested to read [" << offset << "," << offset + size << "] but data is [0,"
        << m_header_buffer_and_size->data_size << "]";
    throw rosbaz::RosBagFormatException(msg.str());
  }

  const auto header = rosbaz::bag_parsing::Header::parse(m_header_buffer_and_size->header_buffer);

  if (header.op != rosbag::OP_MSG_DATA)
  {
    std::stringstream msg;
    msg << "Encountered op=" << static_cast<int>(header.op) << " while deserializing message instead of "
        << static_cast<int>(rosbag::OP_MSG_DATA);
    throw rosbaz::RosBagFormatException(msg.str());
  }

  return m_bag->reader_->read(record_offset + m_header_buffer_and_size->data_offset() + offset, size);
}

//! Size of serialized message
uint32_t MessageInstance::size() const
{
  if (!m_header_buffer_and_size)
  {
    uint64_t record_offset;
    uint32_t record_size;

    getOffsetAndSize(record_offset, record_size);

    m_header_buffer_and_size = m_bag->reader_->read_header_buffer_and_size(record_offset);
  }

  return m_header_buffer_and_size->data_size;
}
}  // namespace rosbaz