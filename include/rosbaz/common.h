#pragma once

#include <rosbaz/internal/nonstd/span.hpp>

namespace rosbaz
{
namespace io
{
using byte = uint8_t;
}  // namespace io

using DataSpan = nonstd::span<const rosbaz::io::byte>;
}  // namespace rosbaz