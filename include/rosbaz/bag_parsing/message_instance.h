#pragma once

#include <cstdint>

#include <ros/console.h>

#include <boost/format.hpp>
#include <boost/make_shared.hpp>
#include <ros/message_traits.h>
#include <ros/serialization.h>
#include <rosbag/constants.h>
#include <rosbag/exceptions.h>
#include <rosbag/structures.h>

#include "rosbaz/bag_parsing/bag.h"
#include "rosbaz/io/reader.h"

namespace rosbaz {
namespace bag_parsing {

class AzBag;
class View;

struct MessageInstance {
 public:
  const ros::Time& getTime() const;
  const std::string& getTopic() const;
  const std::string& getDataType() const;
  const std::string& getMD5Sum() const;
  const std::string& getMessageDefinition() const;

  //! Size of serialized message
  uint32_t size() const;

  template <class T>
  bool isType() const;

  template <class T>
  boost::shared_ptr<T> instantiate() const;

  //! Write serialized message contents out to a stream
  template <typename Stream>
  void write(Stream& stream) const;

 private:
  friend class View;

  MessageInstance(const rosbag::ConnectionInfo& connection_info,
                  const rosbag::IndexEntry& index,
                  const AzBag& bag,
                  rosbaz::io::IReader& reader);

  void getOffsetAndSize(uint64_t& record_offset, uint32_t& record_size) const;

  const rosbag::ConnectionInfo* m_connection_info;
  const rosbag::IndexEntry* m_index_entry;
  const AzBag* m_bag;
  rosbaz::io::IReader* m_reader;
};
}  // namespace bag_parsing
}  // namespace rosbaz

namespace ros {
namespace message_traits {

template <>
struct MD5Sum<rosbaz::bag_parsing::MessageInstance> {
  static const char* value(const rosbaz::bag_parsing::MessageInstance& m) {
    return m.getMD5Sum().c_str();
  }
};

template <>
struct DataType<rosbaz::bag_parsing::MessageInstance> {
  static const char* value(const rosbaz::bag_parsing::MessageInstance& m) {
    return m.getDataType().c_str();
  }
};

template <>
struct Definition<rosbaz::bag_parsing::MessageInstance> {
  static const char* value(const rosbaz::bag_parsing::MessageInstance& m) {
    return m.getMessageDefinition().c_str();
  }
};

}  // namespace message_traits

namespace serialization {

template <>
struct Serializer<rosbaz::bag_parsing::MessageInstance> {
  template <typename Stream>
  inline static void write(Stream& stream, const rosbaz::bag_parsing::MessageInstance& m) {
    m.write(stream);
  }

  inline static uint32_t serializedLength(const rosbaz::bag_parsing::MessageInstance& m) {
    return m.size();
  }
};
}  // namespace serialization
}  // namespace ros


namespace rosbaz {
namespace bag_parsing {

template <class T>
bool MessageInstance::isType() const {
  char const* md5sum = ros::message_traits::MD5Sum<T>::value();
  return md5sum == std::string("*") || md5sum == getMD5Sum();
}

template <class T>
boost::shared_ptr<T> MessageInstance::instantiate() const {
  if (!isType<T>()) {
    return nullptr;
  }

  uint64_t record_offset;
  uint32_t record_size;

  getOffsetAndSize(record_offset, record_size);

  const auto buffer = m_reader->read(record_offset, record_size);
  const rosbaz::DataSpan buffer_span{buffer};

  const auto record = rosbaz::bag_parsing::Record::parse(buffer_span);
  const auto header = rosbaz::bag_parsing::Header::parse(record.header);

  switch (header.op) {
    case rosbag::OP_CONNECTION:
      ROS_WARN(" -> Connection");
      break;

    case rosbag::OP_MSG_DATA: {
      ROS_DEBUG(" -> Msg data");

      const uint32_t connection_id =
          rosbaz::io::read_little_endian<uint32_t>(header.fields.at(rosbag::CONNECTION_FIELD_NAME));

      const auto found_connection = m_bag->connections_.find(connection_id);
      if (found_connection == m_bag->connections_.end()) {
        throw rosbag::BagFormatException(
            (boost::format("Unknown connection ID: %1%") % connection_id).str());
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
    } break;

    case rosbag::OP_INDEX_DATA:
      ROS_WARN("Index data");
      break;

    default:
      ROS_WARN_STREAM("Unkown op " << static_cast<int>(header.op));
      break;
  }


  return nullptr;
}


template <typename Stream>
void MessageInstance::write(Stream& stream) const {
  uint64_t record_offset;
  uint32_t record_size;

  getOffsetAndSize(record_offset, record_size);

  const auto buffer = m_reader->read(record_offset, record_size);
  const rosbaz::DataSpan buffer_span{buffer};

  const auto record = rosbaz::bag_parsing::Record::parse(buffer_span);

  if (record.data_length > 0) {
    std::copy(record.data.begin(), record.data.end(), stream.advance(record.data_length));
  }
}

}  // namespace bag_parsing
}  // namespace rosbaz