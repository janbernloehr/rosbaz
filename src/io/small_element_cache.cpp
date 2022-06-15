#include "rosbaz/io/small_element_cache.h"

namespace rosbaz
{
namespace io
{
SmallElementCacheStrategy::SmallElementCacheStrategy(const size_t element_size, const size_t max_elements)
  : element_size_(element_size), max_elements_(max_elements)
{
}

bool SmallElementCacheStrategy::retrieve(rosbaz::io::byte* buffer, size_t offset, size_t size) const
{
  if (size > element_size_)
  {
    return false;
  }

  auto cache_found = std::find_if(cache_.begin(), cache_.end(), [offset, size](const CacheEntry& entry) {
    return ((entry.offset <= offset) && (offset + size <= entry.offset + entry.data.size()));
  });

  if (cache_found == cache_.end())
  {
    return false;
  }

  assert(offset >= cache_found->offset);

  std::copy_n(cache_found->data.begin() + (offset - cache_found->offset), size, buffer);

  return true;
}

OffsetAndSize SmallElementCacheStrategy::cache_element_offset_and_size(size_t offset, size_t size) const
{
  return OffsetAndSize{ offset, std::max(element_size_, size) };
}

bool SmallElementCacheStrategy::accepts(size_t /* offset */, size_t size) const
{
  return size <= element_size_;
}

void SmallElementCacheStrategy::update(rosbaz::io::Buffer&& data, size_t offset)
{
  if (data.size() > element_size_)
  {
    return;
  }

  cache_.push_back(CacheEntry{ std::move(data), offset });
}

}  // namespace io
}  // namespace rosbaz
