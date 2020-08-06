#pragma once

#include <cstdint>
#include <vector>

#include "rosbaz/common.h"

namespace rosbaz
{
namespace io
{
struct CacheEntry
{
  std::vector<byte> data{};
  std::uint64_t offset{ 0 };
};

}  // namespace io
}  // namespace rosbaz