#include "rosbaz/io/small_element_cache.h"

namespace rosbaz
{
namespace io
{
SmallElementCacheStrategy::SmallElementCacheStrategy(const size_t max_element_size, const size_t max_elements)
  : max_element_size_(max_element_size), max_elements_(max_elements)
{
}

bool SmallElementCacheStrategy::retrieve(rosbaz::io::byte* buffer, size_t offset, size_t count)
{
  if (count > max_element_size_)
  {
    return false;
  }

  auto cache_found = std::find_if(cache_.begin(), cache_.end(), [offset, count](const CacheEntry& entry) {
    return ((entry.offset <= offset) && (offset + count <= entry.offset + entry.data.size()));
  });

  if (cache_found == cache_.end())
  {
    return false;
  }

  assert(offset >= cache_found->offset);

  std::copy_n(cache_found->data.begin() + (offset - cache_found->offset), count, buffer);

  return true;
}

size_t SmallElementCacheStrategy::cache_element_size(size_t count)
{
  return std::max(max_element_size_, count);
}

void SmallElementCacheStrategy::update(rosbaz::io::Buffer&& data, size_t offset)
{
  if (data.size() > max_element_size_)
  {
    return;
  }

  CacheEntry entry;
  entry.offset = offset;
  entry.data = std::move(data);

  cache_.push_back(std::move(entry));
}

}  // namespace io
}  // namespace rosbaz