#include "rosbaz/io/buffer.h"

#include <stdlib.h>

namespace rosbaz
{
namespace io
{
Buffer::Buffer() = default;

Buffer::Buffer(size_t size)
{
  resize(size);
}

Buffer::Buffer(const_pointer begin, const_pointer end)
{
  resize(end - begin);
  std::copy(begin, end, buffer_);
}

Buffer::Buffer(const Buffer& other)
{
  resize(other.size());
  std::copy(other.begin(), other.end(), buffer_);
}

Buffer& Buffer::operator=(const Buffer& other)
{
  resize(other.size());
  std::copy(other.begin(), other.end(), buffer_);
  return *this;
}

Buffer::Buffer(Buffer&& other) : buffer_{ other.buffer_ }, capacity_{ other.capacity_ }, size_{ other.size_ }
{
  other.buffer_ = nullptr;
  other.capacity_ = 0;
  other.size_ = 0;
}

Buffer& Buffer::operator=(Buffer&& other)
{
  buffer_ = other.buffer_;
  capacity_ = other.capacity_;
  size_ = other.size_;

  other.buffer_ = nullptr;
  other.capacity_ = 0;
  other.size_ = 0;
  return *this;
}

Buffer::~Buffer()
{
  if (buffer_)
  {
    free(buffer_);
    buffer_ = nullptr;
  }
}

Buffer::iterator Buffer::begin()
{
  return buffer_;
}
Buffer::const_iterator Buffer::begin() const
{
  return buffer_;
}

Buffer::iterator Buffer::end()
{
  return buffer_ + size_;
}
Buffer::const_iterator Buffer::end() const
{
  return buffer_ + size_;
}

const rosbaz::io::byte* Buffer::data() const
{
  return buffer_;
}

rosbaz::io::byte* Buffer::data()
{
  return buffer_;
}
size_t Buffer::capacity() const
{
  return capacity_;
}
size_t Buffer::size() const
{
  return size_;
}

void Buffer::resize(size_t size)
{
  size_ = size;
  ensureCapacity(size);
}

void Buffer::ensureCapacity(size_t capacity)
{
  if (capacity <= capacity_)
    return;

  if (capacity_ == 0)
    capacity_ = capacity;
  else
  {
    while (capacity_ < capacity)
      capacity_ *= 2;
  }

  buffer_ = reinterpret_cast<rosbaz::io::byte*>(realloc(buffer_, capacity_));
}

}  // namespace io
}  // namespace rosbaz