#pragma once

#include "rosbaz/io/writer.h"
#include "ros/common.h"

#include <cstring>

namespace rosbaz
{
namespace bag_writing
{

void writeHeader(rosbaz::io::Block& block, ros::M_string const& fields);

void writeDataLength(rosbaz::io::Block& block, uint32_t data_len);

void appendHeaderToBuffer(Buffer& buf, ros::M_string const& fields);

void appendDataLengthToBuffer(Buffer& buf, uint32_t data_len);

}  // namespace bag_writing
}  // namespace rosbaz