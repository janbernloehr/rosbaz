#pragma once
#include <string>

namespace ros
{
class Time;
}  // namespace ros

namespace rosbaz
{
namespace bag_writing
{
template <typename T>
std::string toHeaderString(T const* field)
{
  return std::string(reinterpret_cast<const char*>(field), sizeof(T));
}

std::string toHeaderString(ros::Time const* field);

}  // namespace bag_writing
}  // namespace rosbaz