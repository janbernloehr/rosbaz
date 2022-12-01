#pragma once

#include <cstdint>
#include <ros/datatypes.h>

namespace rosbaz
{
namespace io
{
class Block;
}  // namespace io

namespace bag_writing
{
void writeHeader(rosbaz::io::Block& block, ros::M_string const& fields);

void writeDataLength(rosbaz::io::Block& block, std::uint32_t data_len);

}  // namespace bag_writing
}  // namespace rosbaz