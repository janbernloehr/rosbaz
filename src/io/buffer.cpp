#include "rosbaz/io/buffer.h"

#include <cassert>
#include <cstdlib>

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
  resize(std::distance(begin, end));
  std::copy(begin, end, this->begin());
}

Buffer::Buffer(const Buffer& other)
{
  resize(other.size());
  std::copy(other.begin(), other.end(), begin());
}

Buffer& Buffer::operator=(const Buffer& other)
{
  resize(other.size());
  std::copy(other.begin(), other.end(), begin());
  return *this;
}

Buffer::Buffer(Buffer&& other)
  : buffer_{ std::exchange(other.buffer_, nullptr) }
  , capacity_{ std::exchange(other.capacity_, 0) }
  , offset_{ std::exchange(other.offset_, 0) }
  , size_{ std::exchange(other.size_, 0) }
{
}

Buffer& Buffer::operator=(Buffer&& other)
{
  if (buffer_)
  {
    std::free(buffer_);
  }
  buffer_ = std::exchange(other.buffer_, nullptr);
  capacity_ = std::exchange(other.capacity_, 0);
  offset_ = std::exchange(other.offset_, 0);
  size_ = std::exchange(other.size_, 0);
  return *this;
}

Buffer::~Buffer()
{
  if (buffer_)
  {
    std::free(buffer_);
    buffer_ = nullptr;
  }
}

Buffer::iterator Buffer::begin()
{
  return buffer_ + offset_;
}
Buffer::const_iterator Buffer::begin() const
{
  return buffer_ + offset_;
}

Buffer::iterator Buffer::end()
{
  return buffer_ + offset_ + size_;
}
Buffer::const_iterator Buffer::end() const
{
  return buffer_ + offset_ + size_;
}

const rosbaz::io::byte* Buffer::data() const
{
  return buffer_ + offset_;
}

rosbaz::io::byte* Buffer::data()
{
  return buffer_ + offset_;
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
  if (offset_ != 0)
  {
    std::copy_n(begin(), size_, buffer_);
    offset_ = 0;
  }
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

  buffer_ = reinterpret_cast<rosbaz::io::byte*>(std::realloc(buffer_, capacity_));
  assert(buffer_ != nullptr);
}

void Buffer::shrinkTo(size_type offset, size_type size)
{
  assert(offset + size <= this->size());
  offset_ += offset;
  size_ = size;
}

}  // namespace io
}  // namespace rosbaz
