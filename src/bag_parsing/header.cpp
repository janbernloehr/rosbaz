#include "rosbaz/bag_parsing/header.h"

#include <ros/console.h>
#include <rosbag/constants.h>

#include "rosbaz/common.h"
#include "rosbaz/exceptions.h"
#include "rosbaz/io/io_helpers.h"

namespace rosbaz
{
namespace bag_parsing
{
Header Header::parse(rosbaz::DataSpan source)
{
  Header header;
  std::uint64_t offset = 0;

  while (offset < source.size())
  {
    const std::uint32_t field_length =
        rosbaz::io::read_little_endian<std::uint32_t>(source.subspan(offset, sizeof(std::uint32_t)));
    offset += sizeof(std::uint32_t);

    const auto field_span = source.subspan(offset, field_length);

    const rosbaz::io::byte* value_pos = std::find(field_span.begin(), field_span.end(), rosbag::FIELD_DELIM);
    if (value_pos == field_span.end())
    {
      std::stringstream msg;
      msg << "Could not find field delimiter '" << rosbag::FIELD_DELIM << "' in field data";
      throw rosbaz::RosBagFormatException(msg.str());
    }
    const size_t name_length = std::distance(field_span.begin(), value_pos);
    const std::string field_name = rosbaz::io::to_string(field_span.subspan(0, name_length));

    assert(field_length > name_length);
    const std::uint32_t data_length = field_length - static_cast<uint32_t>(name_length) - 1;

    ROS_DEBUG_STREAM("field: " << field_name << " data length: " << data_length);

    const auto data = header.fields.insert({ field_name, field_span.subspan(name_length + 1, data_length) });
    offset += field_length;

    if (field_name == rosbag::OP_FIELD_NAME)
    {
      header.op = rosbaz::io::read_little_endian<uint8_t>(data.first->second);
    }
  }

  return header;
}
}  // namespace bag_parsing
}  // namespace rosbaz