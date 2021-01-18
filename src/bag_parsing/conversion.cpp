#include "rosbaz/bag_parsing/conversion.h"

#include <ros/console.h>
#include <rosbag/constants.h>
#include <boost/make_shared.hpp>

#include "rosbaz/exceptions.h"
#include "rosbaz/io/io_helpers.h"

namespace rosbaz
{
namespace bag_parsing
{
ros::Time unpack_time(uint64_t packed_time)
{
  ros::Time data;

  uint64_t bitmask = (1LL << 33) - 1;
  data.sec = static_cast<uint32_t>(packed_time & bitmask);
  data.nsec = static_cast<uint32_t>(packed_time >> 32);

  return data;
}

bool verifyVersionHeader(rosbaz::DataSpan header)
{
  std::string expected = "#ROSBAG V2.0\n";
  std::string actual = rosbaz::io::to_string(header);

  return expected == actual;
}

rosbag::ChunkHeader as_chunk_header(const Header& header, uint32_t compressed_size)
{
  if (header.op != rosbag::OP_CHUNK)
  {
    std::stringstream msg;
    msg << "Cannot create ChunkHeader from op=" << static_cast<int>(header.op)
        << " != " << static_cast<int>(rosbag::OP_CHUNK) << ".";
    throw rosbaz::RosBagFormatException(msg.str());
  }

  rosbag::ChunkHeader chunk_header;

  const auto& compression_field = header.fields.at(rosbag::COMPRESSION_FIELD_NAME);
  chunk_header.compression = rosbaz::io::to_string(compression_field);

  chunk_header.compressed_size = compressed_size;
  chunk_header.uncompressed_size = header.read_field_little_endian<uint32_t>(rosbag::SIZE_FIELD_NAME);

  return chunk_header;
}

rosbag::ConnectionInfo as_connection_info(const Header& header, rosbaz::DataSpan data)
{
  if (header.op != rosbag::OP_CONNECTION)
  {
    std::stringstream msg;
    msg << "Cannot create ConnectionInfo from op=" << static_cast<int>(header.op)
        << " != " << static_cast<int>(rosbag::OP_CONNECTION) << ".";
    throw rosbaz::RosBagFormatException(msg.str());
  }

  rosbag::ConnectionInfo info;

  info.id = header.read_field_little_endian<uint32_t>(rosbag::CONNECTION_FIELD_NAME);

  const auto& topic_field = header.fields.at(rosbag::TOPIC_FIELD_NAME);
  info.topic = rosbaz::io::to_string(topic_field);

  const auto connection_header = Header::parse(data);

  // topic, type, md5sum, message_definition

  const auto& type_field = connection_header.fields.at("type");
  info.datatype = rosbaz::io::to_string(type_field);

  const auto& md5_field = connection_header.fields.at("md5sum");
  info.md5sum = rosbaz::io::to_string(md5_field);

  const auto& msg_def_field = connection_header.fields.at("message_definition");
  info.msg_def = rosbaz::io::to_string(msg_def_field);

  info.header = boost::make_shared<ros::M_string>();

  for (const auto& field : connection_header.fields)
  {
    (*info.header)[field.first] = rosbaz::io::to_string(field.second);
  }

  return info;
}

rosbag::ChunkInfo as_chunk_info(const Header& header, rosbaz::DataSpan data)
{
  if (header.op != rosbag::OP_CHUNK_INFO)
  {
    std::stringstream msg;
    msg << "Cannot create ChunkInfo from op=" << static_cast<int>(header.op)
        << " != " << static_cast<int>(rosbag::OP_CHUNK_INFO) << ".";
    throw rosbaz::RosBagFormatException(msg.str());
  }

  rosbag::ChunkInfo info;

  auto ver = header.read_field_little_endian<uint32_t>(rosbag::VER_FIELD_NAME);

  if (ver != rosbag::CHUNK_INFO_VERSION)
  {
    std::stringstream msg;
    msg << "Unsupported chunk info version " << ver << ". Only " << rosbag::CHUNK_INFO_VERSION << " is supported.";
    throw rosbaz::UnsupportedRosBagException(msg.str());
  }

  info.pos = header.read_field_little_endian<uint64_t>(rosbag::CHUNK_POS_FIELD_NAME);

  info.start_time = unpack_time(header.read_field_little_endian<uint64_t>(rosbag::START_TIME_FIELD_NAME));
  info.end_time = unpack_time(header.read_field_little_endian<uint64_t>(rosbag::END_TIME_FIELD_NAME));

  ROS_DEBUG_STREAM(" pos: " << info.pos << " start: " << info.start_time.toNSec()
                            << " end: " << info.end_time.toNSec());

  auto count = header.read_field_little_endian<uint32_t>(rosbag::COUNT_FIELD_NAME);

  for (uint32_t i = 0; i < count; ++i)
  {
    const auto conn_id = rosbaz::io::read_little_endian<uint32_t>(data.subspan(8u * i, 4));
    const auto conn_count = rosbaz::io::read_little_endian<uint32_t>(data.subspan(8u * i + 4u, 4));

    info.connection_counts[conn_id] = conn_count;
    ROS_DEBUG_STREAM(" id: " << conn_id << " count: " << conn_count);
  }

  return info;
}

}  // namespace bag_parsing
}  // namespace rosbaz