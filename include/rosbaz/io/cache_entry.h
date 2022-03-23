#pragma once

#include <cstdint>
#include <vector>

#include "rosbaz/common.h"
#include "rosbaz/io/buffer.h"

namespace rosbaz
{
namespace io
{
struct CacheEntry
{
  rosbaz::io::Buffer data{};
  std::uint64_t offset{ 0 };
};

}  // namespace io
}  // namespace rosbaz