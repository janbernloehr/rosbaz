#pragma once

#include "rosbaz/common.h"

#include <cassert>
#include <cstddef>
#include <fstream>
#include <limits>
#include <string>
#include <type_traits>

namespace rosbaz
{
namespace io
{
using RosStream = std::ifstream;

/// Read \p size many bytes from \p ifs into \p target.
void read_from(RosStream& ifs, byte* target, const size_t size);

/// Convert the given byte span into a string assuming the span is not null
/// terminated.
inline std::string to_string(rosbaz::DataSpan data)
{
  return std::string{ reinterpret_cast<const char*>(data.begin()), data.size() };
}

/// Deserialize a T at \p offset from \p buffer.
template <class T>
T read_little_endian(DataSpan buffer, size_t offset = 0)
{
  static_assert(std::is_standard_layout<T>::value == true, "T must have standard layout.");
  assert(buffer.size() >= offset + sizeof(T));
  return *reinterpret_cast<const T*>(buffer.begin() + offset);
}

}  // namespace io

}  // namespace rosbaz
