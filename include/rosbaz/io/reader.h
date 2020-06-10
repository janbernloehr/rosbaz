#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "rosbaz/common.h"
#include "rosbaz/io/io_helpers.h"

namespace rosbaz {

namespace io {

struct HeaderBufferAndSize {
  std::vector<rosbaz::io::byte> header_buffer;
  uint32_t header_size;
  uint32_t data_size;
};

class IReader {
 public:
  virtual ~IReader() = default;

  virtual size_t size() = 0;

  std::vector<rosbaz::io::byte> read(size_t offset, size_t count);

  template <size_t N>
  std::array<rosbaz::io::byte, N> read(size_t offset) {
    std::array<rosbaz::io::byte, N> buffer;

    read_fixed(buffer.begin(), offset, N);

    return buffer;
  }

  template <class T>
  T read_little_endian(size_t offset = 0) {
    auto buffer = read<sizeof(T)>(offset);
    return *reinterpret_cast<const T*>(buffer.begin());
  }

  HeaderBufferAndSize read_header_buffer_and_size(size_t offset);

 private:
  virtual void read_fixed(rosbaz::io::byte* buffer, size_t offset, size_t count) = 0;
};

}  // namespace io
}  // namespace rosbaz