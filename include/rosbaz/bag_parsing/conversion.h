#pragma once

#include <cstdint>
#include <string>

#include <ros/time.h>
#include <rosbag/structures.h>

#include "rosbaz/bag_parsing/header.h"
#include "rosbaz/common.h"

namespace rosbaz {
namespace bag_parsing {
ros::Time unpack_time(uint64_t packed_time);

rosbag::ChunkHeader as_chunk_header(const Header& header, uint32_t compressed_size);

rosbag::ConnectionInfo as_connection_info(const Header& header, rosbaz::DataSpan data);

rosbag::ChunkInfo as_chunk_info(const Header& header, rosbaz::DataSpan data);

bool verifyVersionHeader(rosbaz::DataSpan header);

}  // namespace bag_parsing
}  // namespace rosbaz