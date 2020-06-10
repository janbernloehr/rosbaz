#pragma once

#include <array>
#include <cassert>
#include <fstream>
#include <string>
#include <vector>

#include "rosbaz/common.h"

namespace rosbaz {

namespace io {

using RosStream = std::ifstream; // std::basic_ifstream<byte>;

size_t size_of_file(std::string file_path);

void read_from(RosStream &ifs, byte *target, const size_t count);

void read_from(RosStream &ifs, std::vector<byte> &target, const size_t count);

template <size_t N>
void read_from(RosStream &ifs, std::array<byte, N> &target) {
  read_from(ifs, target.begin(), N);
}

inline std::string to_string(rosbaz::DataSpan data) {
  return std::string{reinterpret_cast<const char *>(data.begin()),
                     static_cast<size_t>(data.size())};
}

template <class T> T read_little_endian(DataSpan buffer, size_t offset = 0) {
  assert(static_cast<size_t>(buffer.size()) >= offset + sizeof(T));
  return *reinterpret_cast<const T *>(buffer.begin() + offset);
}

template <class T> T read_little_endian(RosStream &ifs) {
  std::array<rosbaz::io::byte, sizeof(T)> buffer;

  read_from(ifs, buffer);

  return read_little_endian<T>(buffer);
}

} // namespace io

} // namespace rosbaz