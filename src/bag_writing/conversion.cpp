#include "rosbaz/bag_writing/conversion.h"

namespace rosbaz
{
namespace bag_writing
{

std::string toHeaderString(ros::Time const* field)
{
  uint64_t packed_time = ((static_cast<uint64_t>(field->nsec)) << 32) + field->sec;
  return toHeaderString(&packed_time);
}

}  // namespace bag_writing
}  // namespace rosbaz