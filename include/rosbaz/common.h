#pragma once

#include <rosbaz/internal/nonstd/span.hpp>

#include <vector>

namespace rosbaz
{
namespace io
{
using byte = uint8_t;
}  // namespace io

using DataSpan = nonstd::span<const rosbaz::io::byte>;

using Buffer = std::vector<rosbaz::io::byte>;
}  // namespace rosbaz