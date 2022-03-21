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

}  // namespace bag_writing
}  // namespace rosbaz