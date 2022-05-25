#include "rosbaz/io/hybrid_element_cache.h"

namespace rosbaz
{
namespace io
{
HybridElementCacheStrategy::HybridElementCacheStrategy(const size_t max_small_element_size,
                                                       const size_t max_small_elements, size_t max_large_elements)
  : max_small_element_size_(max_small_element_size)
  , max_small_elements_(max_small_elements)
  , max_large_elements_(max_large_elements)
{
}

bool HybridElementCacheStrategy::retrieve(rosbaz::io::byte* buffer, size_t offset, size_t size) const
{
  if (size > max_small_element_size_)
  {
    auto large_cache_found =
        std::find_if(large_element_cache_.begin(), large_element_cache_.end(), [offset, size](const CacheEntry& entry) {
          return ((entry.offset <= offset) && (offset + size <= entry.offset + entry.data.size()));
        });

    if (large_cache_found == large_element_cache_.end())
    {
      return false;
    }

    assert(offset >= large_cache_found->offset);

    std::copy_n(large_cache_found->data.begin() + (offset - large_cache_found->offset), size, buffer);

    return true;
  }
  else
  {
    auto small_cache_found =
        std::find_if(small_element_cache_.begin(), small_element_cache_.end(), [offset, size](const CacheEntry& entry) {
          return ((entry.offset <= offset) && (offset + size <= entry.offset + entry.data.size()));
        });

    if (small_cache_found == small_element_cache_.end())
    {
      return false;
    }

    assert(offset >= small_cache_found->offset);

    std::copy_n(small_cache_found->data.begin() + (offset - small_cache_found->offset), size, buffer);

    return true;
  }
}

OffsetAndSize HybridElementCacheStrategy::cache_element_offset_and_size(size_t offset, size_t size) const
{
  return OffsetAndSize{ offset, std::max(max_small_element_size_, size) };
}

bool HybridElementCacheStrategy::accepts(size_t, size_t) const
{
  return true;
}

void HybridElementCacheStrategy::update(rosbaz::io::Buffer&& data, size_t offset)
{
  CacheEntry entry;
  entry.offset = offset;
  entry.data = std::move(data);

  if (entry.data.size() > max_small_element_size_)
  {
    large_element_cache_.push_back(std::move(entry));

    if (large_element_cache_.size() > max_large_elements_)
    {
      large_element_cache_.erase(large_element_cache_.begin());
    }
  }
  else
  {
    small_element_cache_.push_back(std::move(entry));
  }
}
}  // namespace io
}  // namespace rosbaz
