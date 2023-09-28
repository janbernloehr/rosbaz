#pragma once

#include "rosbaz/internal/span.hpp"

#include <vector>

namespace rosbaz
{
namespace io
{
using byte = uint8_t;
}  // namespace io

using DataSpan = nonstd::span<const rosbaz::io::byte>;

}  // namespace rosbaz
