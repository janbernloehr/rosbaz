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

void ICacheStrategy::use_cache_hints(const std::vector<uint64_t>&)
{
}

}  // namespace io
}  // namespace rosbaz
