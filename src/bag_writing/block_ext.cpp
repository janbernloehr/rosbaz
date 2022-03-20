#include "rosbaz/bag_writing/block_ext.h"

#include <ros/header.h>

namespace rosbaz
{
namespace bag_writing
{
void writeHeader(rosbaz::io::Block& block, ros::M_string const& fields)
{
  boost::shared_array<uint8_t> header_buffer;
  uint32_t header_len;
  ros::Header::write(fields, header_buffer, header_len);
  block.write(reinterpret_cast<const rosbaz::io::byte*>(&header_len), 4);
  block.write(reinterpret_cast<const rosbaz::io::byte*>(header_buffer.get()), header_len);
}

void writeDataLength(rosbaz::io::Block& block, uint32_t data_len)
{
  block.write(reinterpret_cast<const rosbaz::io::byte*>(&data_len), 4);
}

void appendHeaderToBuffer(Buffer& buf, ros::M_string const& fields)
{
  boost::shared_array<uint8_t> header_buffer;
  uint32_t header_len;
  ros::Header::write(fields, header_buffer, header_len);

  size_t offset = buf.size();

  buf.resize(buf.size() + 4 + header_len);

  memcpy(buf.data() + offset, &header_len, 4);
  offset += 4;
  memcpy(buf.data() + offset, header_buffer.get(), header_len);
}

void appendDataLengthToBuffer(Buffer& buf, uint32_t data_len)
{
  size_t offset = buf.size();

  buf.resize(buf.size() + 4);

  memcpy(buf.data() + offset, &data_len, 4);
}

}  // namespace bag_writing
}  // namespace rosbaz