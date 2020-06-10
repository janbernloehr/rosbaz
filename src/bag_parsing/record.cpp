#include "rosbaz/bag_parsing/record.h"

#include <cstdint>
#include <ros/console.h>

#include "rosbaz/common.h"
#include "rosbaz/io/io_helpers.h"

namespace rosbaz {
namespace bag_parsing {

Record Record::parse(rosbaz::DataSpan source) {
  Record record;

  assert(source.size() >= sizeof(uint32_t));
  record.header_length = rosbaz::io::read_little_endian<std::uint32_t>(source);

  record.header = source.subspan(sizeof(std::uint32_t), record.header_length);

  assert(source.size() >=
         sizeof(uint32_t) + record.header_length + sizeof(uint32_t));
  record.data_length = rosbaz::io::read_little_endian<std::int32_t>(
      source, sizeof(std::uint32_t) + record.header_length);

  ROS_DEBUG_STREAM("record: header length: "
                   << record.header_length << " data length: "
                   << record.data_length << " source: " << source.size());

  record.data = source.subspan(+sizeof(std::uint32_t) +
                                   static_cast<size_t>(record.header_length) +
                                   sizeof(std::uint32_t),
                               static_cast<int32_t>(record.data_length));

  return record;
}
} // namespace bag_parsing
} // namespace rosbaz