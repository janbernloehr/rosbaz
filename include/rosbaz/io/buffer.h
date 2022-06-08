#pragma once

#include "rosbaz/common.h"

#include <cstdint>
#include <cstddef>

namespace rosbaz
{
namespace io
{
class Buffer
{
public:
  using value_type = rosbaz::io::byte;
  using size_type = size_t;
  using pointer = rosbaz::io::byte*;
  using const_pointer = const rosbaz::io::byte*;
  using iterator = rosbaz::io::byte*;
  using const_iterator = const rosbaz::io::byte*;

  Buffer();
  explicit Buffer(size_type size);
  Buffer(const_pointer begin, const_pointer end);

  Buffer(const Buffer&);
  Buffer& operator=(const Buffer&);

  Buffer(Buffer&&);
  Buffer& operator=(Buffer&&);

  ~Buffer();

  iterator begin();
  const_iterator begin() const;

  iterator end();
  const_iterator end() const;

  const rosbaz::io::byte* data() const;
  rosbaz::io::byte* data();
  size_type capacity() const;
  size_type size() const;

  void resize(size_type size);

  /// Set internal offset and size without any data movement so that
  /// the buffer's begin points to \p offset and the buffer's size is
  /// \p size.
  void shrinkTo(size_type offset, size_type size);

private:
  void ensureCapacity(size_type capacity);

private:
  rosbaz::io::byte* buffer_{ nullptr };
  size_type capacity_{ 0 };
  size_type offset_{ 0 };
  size_type size_{ 0 };
};

}  // namespace io
}  // namespace rosbaz
