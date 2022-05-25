#include "rosbaz/io/cache_strategy.h"

namespace rosbaz
{
namespace io
{
bool OffsetAndSize::contains(size_t _offset, size_t _size) const
{
  return (offset <= _offset) && (offset + size >= _offset + _size);
}

bool operator==(const OffsetAndSize& lhs, const OffsetAndSize& rhs)
{
  return (lhs.offset == rhs.offset) && (lhs.size == rhs.size);
}

bool operator!=(const OffsetAndSize& lhs, const OffsetAndSize& rhs)
{
  return !(lhs == rhs);
}

void ICacheStrategy::set_cache_hints(const nonstd::span<uint64_t>)
{
}

}  // namespace io
}  // namespace rosbaz
