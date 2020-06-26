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

bool HybridElementCacheStrategy::retrieve(rosbaz::io::byte* buffer, size_t offset, size_t count)
{
  if (count > max_small_element_size_)
  {
    auto large_cache_found = std::find_if(
        large_element_cache_.begin(), large_element_cache_.end(), [offset, count](const CacheEntry& entry) {
          return ((entry.offset <= offset) && (offset + count <= entry.offset + entry.data.size()));
        });

    if (large_cache_found == large_element_cache_.end())
    {
      return false;
    }

    std::copy_n(large_cache_found->data.begin() + (offset - large_cache_found->offset), count, buffer);

    return true;
  }
  else
  {
    auto small_cache_found = std::find_if(
        small_element_cache_.begin(), small_element_cache_.end(), [offset, count](const CacheEntry& entry) {
          return ((entry.offset <= offset) && (offset + count <= entry.offset + entry.data.size()));
        });

    if (small_cache_found == small_element_cache_.end())
    {
      return false;
    }

    std::copy_n(small_cache_found->data.begin() + (offset - small_cache_found->offset), count, buffer);

    return true;
  }
}

size_t HybridElementCacheStrategy::cache_element_size(size_t count)
{
  return std::max(max_small_element_size_, count);
}

void HybridElementCacheStrategy::update(std::vector<rosbaz::io::byte> data, size_t offset)
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