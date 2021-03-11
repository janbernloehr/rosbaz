#pragma once

#include <ros/console.h>
#include <ros/message_traits.h>
#include <ros/serialization.h>
#include <rosbag/constants.h>
#include <rosbag/exceptions.h>
#include <rosbag/structures.h>

#include <boost/format.hpp>
#include <boost/make_shared.hpp>
#include <boost/optional.hpp>
#include <cstdint>
#include <string>

#include "rosbaz/bag.h"
#include "rosbaz/exceptions.h"
#include "rosbaz/io/reader.h"

namespace rosbaz
{
class Bag;
struct View;

struct MessageInstance
{
public:
  const ros::Time& getTime() const;
  const std::string& getTopic() const;
  const std::string& getDataType() const;
  const std::string& getMD5Sum() const;
  const std::string& getMessageDefinition() const;

  boost::shared_ptr<ros::M_string> getConnectionHeader() const;

  std::string getCallerId() const;
  bool isLatching() const;

  //! Size of serialized message
  uint32_t size() const;

  //! Test whether the underlying message of the specified type.
  /*!
   * returns true iff the message is of the template type
   */
  template <class T>
  bool isType() const;

  //! Templated call to instantiate a message
  /*!
   * returns NULL pointer if incompatible
   */
  template <class T>
  boost::shared_ptr<T> instantiate() const;

  //! Templated call to instantiate a ros message from a subset of the raw message
  /*!
   * returns NULL pointer if incompatible
   */
  template <class T>
  boost::shared_ptr<T> instantiate_subset(uint32_t offset, uint32_t size) const;

  std::vector<rosbaz::io::byte> read_subset(uint32_t offset, uint32_t size) const;

  //! Write serialized message contents out to a stream
  template <typename Stream>
  void write(Stream& stream) const;

private:
  friend struct View;

  MessageInstance(const rosbag::ConnectionInfo& connection_info, const rosbag::IndexEntry& index, const Bag& bag);

  void getOffsetAndSize(uint64_t& record_offset, uint32_t& record_size) const;

  const rosbag::ConnectionInfo* m_connection_info{ nullptr };
  const rosbag::IndexEntry* m_index_entry{ nullptr };
  const Bag* m_bag{ nullptr };

  mutable boost::optional<rosbaz::io::HeaderBufferAndSize> m_header_buffer_and_size{};
};
}  // namespace rosbaz

namespace ros
{
namespace message_traits
{
template <>
struct MD5Sum<rosbaz::MessageInstance>
{
  static const char* value(const rosbaz::MessageInstance& m)
  {
    return m.getMD5Sum().c_str();
  }
};

template <>
struct DataType<rosbaz::MessageInstance>
{
  static const char* value(const rosbaz::MessageInstance& m)
  {
    return m.getDataType().c_str();
  }
};

template <>
struct Definition<rosbaz::MessageInstance>
{
  static const char* value(const rosbaz::MessageInstance& m)
  {
    return m.getMessageDefinition().c_str();
  }
};

}  // namespace message_traits

namespace serialization
{
template <>
struct Serializer<rosbaz::MessageInstance>
{
  template <typename Stream>
  inline static void write(Stream& stream, const rosbaz::MessageInstance& m)
  {
    m.write(stream);
  }

  inline static uint32_t serializedLength(const rosbaz::MessageInstance& m)
  {
    return m.size();
  }
};
}  // namespace serialization
}  // namespace ros

namespace rosbaz
{
template <class T>
bool MessageInstance::isType() const
{
  char const* md5sum = ros::message_traits::MD5Sum<T>::value();
  return md5sum == std::string("*") || md5sum == getMD5Sum();
}

template <class T>
boost::shared_ptr<T> MessageInstance::instantiate() const
{
  if (!isType<T>())
  {
    return nullptr;
  }

  uint64_t record_offset;
  uint32_t record_size;

  getOffsetAndSize(record_offset, record_size);

  const auto buffer = m_bag->reader_->read(record_offset, record_size);
  const rosbaz::DataSpan buffer_span{ buffer };

  const auto record = rosbaz::bag_parsing::Record::parse(buffer_span);
  const auto header = rosbaz::bag_parsing::Header::parse(record.header);

  if (header.op != rosbag::OP_MSG_DATA)
  {
    std::stringstream msg;
    msg << "Encountered op=" << static_cast<int>(header.op) << " while deserializing message instead of "
        << static_cast<int>(rosbag::OP_MSG_DATA);
    throw rosbaz::RosBagFormatException(msg.str());
  }

  const uint32_t connection_id = header.read_field_little_endian<uint32_t>(rosbag::CONNECTION_FIELD_NAME);

  const auto found_connection = m_bag->connections_.find(connection_id);
  if (found_connection == m_bag->connections_.end())
  {
    throw rosbag::BagFormatException((boost::format("Unknown connection ID: %1%") % connection_id).str());
  }
  auto ptr = boost::make_shared<T>();

  ros::serialization::PreDeserializeParams<T> predes_params;
  predes_params.message = ptr;
  predes_params.connection_header = found_connection->second.header;
  ros::serialization::PreDeserialize<T>::notify(predes_params);

  ros::serialization::IStream s(const_cast<uint8_t*>(&(*record.data.begin())),
                                static_cast<uint32_t>(record.data.size()));
  ros::serialization::deserialize(s, *ptr);

  return ptr;
}

template <class T>
boost::shared_ptr<T> MessageInstance::instantiate_subset(uint32_t offset, uint32_t size) const
{
  const auto subset_buffer = read_subset(offset, size);

  auto ptr = boost::make_shared<T>();

  ros::serialization::IStream s(const_cast<uint8_t*>(&(*subset_buffer.begin())),
                                static_cast<uint32_t>(subset_buffer.size()));
  ros::serialization::deserialize(s, *ptr);
}

template <typename Stream>
void MessageInstance::write(Stream& stream) const
{
  uint64_t record_offset;
  uint32_t record_size;

  getOffsetAndSize(record_offset, record_size);

  const auto buffer = m_bag->reader_->read(record_offset, record_size);
  const rosbaz::DataSpan buffer_span{ buffer };

  const auto record = rosbaz::bag_parsing::Record::parse(buffer_span);

  if (record.data_length > 0)
  {
    std::copy(record.data.begin(), record.data.end(), stream.advance(record.data_length));
  }
}

}  // namespace rosbaz