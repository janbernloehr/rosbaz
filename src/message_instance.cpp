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

uint32_t MessageInstance::getConnectionId() const
{
  return m_connection_info->id;
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

  const auto& found_chunk = m_bag->chunk_exts_lookup_.at(m_index_entry->chunk_pos);
  const auto& found_size = found_chunk->message_records.at(m_index_entry->offset);

  record_offset = found_chunk->data_offset + m_index_entry->offset;
  record_size = found_size.data_size;
}

rosbaz::io::Buffer MessageInstance::read() const
{
  uint64_t record_offset;
  uint32_t record_size;

  getOffsetAndSize(record_offset, record_size);

  ROS_DEBUG_STREAM("Reading full record to instantiate message. record_offset: " << record_offset
                                                                                 << " record_size: " << record_size);

  rosbaz::io::Buffer record_buffer = m_bag->reader_->read(record_offset, record_size);

  if (!m_header_buffer_and_size)
  {
    m_header_buffer_and_size = rosbaz::io::parseHeaderBufferAndSize(rosbaz::DataSpan{ record_buffer });
  }

  const auto header = rosbaz::bag_parsing::Header::parse(m_header_buffer_and_size->header_buffer);

  if (header.op != rosbag::OP_MSG_DATA)
  {
    std::stringstream msg;
    msg << "Encountered op=" << static_cast<int>(header.op) << " while deserializing message instead of "
        << static_cast<int>(rosbag::OP_MSG_DATA);
    throw rosbaz::RosBagFormatException(msg.str());
  }

  record_buffer.shrinkTo(m_header_buffer_and_size->data_offset(), m_header_buffer_and_size->data_size);

  return record_buffer;
}

rosbaz::io::Buffer MessageInstance::read_subset(uint32_t offset, uint32_t size) const
{
  uint64_t record_offset;
  uint32_t record_size;

  getOffsetAndSize(record_offset, record_size);

  if (!m_header_buffer_and_size)
  {
    ROS_DEBUG_STREAM("Reading record header at offset " << record_offset);
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

  ROS_DEBUG_STREAM("Reading record to instantiate message. record_offset: "
                   << record_offset << " data_offset: " << m_header_buffer_and_size->data_offset()
                   << " offset: " << offset << " size: " << size);
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

    ROS_DEBUG_STREAM("Reading record header at offset " << record_offset);
    m_header_buffer_and_size = m_bag->reader_->read_header_buffer_and_size(record_offset);
  }

  return m_header_buffer_and_size->data_size;
}
}  // namespace rosbaz
