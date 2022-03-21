#pragma once
#include "ros/time.h"
#include <string>

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