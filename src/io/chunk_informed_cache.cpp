#include "rosbaz/io/chunk_informed_cache.h"

#include <sstream>
#include <iostream>
namespace rosbaz
{
namespace io
{
ChunkInformedCacheStrategy::ChunkInformedCacheStrategy(const size_t max_element_size, const size_t max_elements)
  : max_element_size_(max_element_size), max_elements_(max_elements)
{
}

bool ChunkInformedCacheStrategy::retrieve(rosbaz::io::byte* buffer, size_t offset, size_t count)
{
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

void ChunkInformedCacheStrategy::use_cache_hints(const std::vector<uint64_t>& cache_hints)
{
  cache_hints_ = cache_hints;
}

OffsetAndSize ChunkInformedCacheStrategy::cache_element_offset_and_size(size_t offset, size_t count) const
{
  if (cache_hints_.empty())
  {
    return OffsetAndSize{ offset, std::max(max_element_size_, count) };
  }
  else
  {
    auto begin_it = std::upper_bound(cache_hints_.begin(), cache_hints_.end(), offset);
    size_t begin = cache_hints_.front();
    if (begin_it != cache_hints_.end())
    {
      --begin_it;
      begin = *begin_it;
    }

    auto end_it =
        std::lower_bound(cache_hints_.begin(), cache_hints_.end(), offset + std::max(max_element_size_, count));
    size_t end = cache_hints_.back();
    if (end_it != cache_hints_.end())
    {
      end = *end_it;
    }

    return OffsetAndSize{ begin, end - begin };
  }
}

bool ChunkInformedCacheStrategy::accepts(size_t /* offset */, size_t count) const
{
  return count <= max_element_size_;
}

void ChunkInformedCacheStrategy::update(rosbaz::io::Buffer&& data, size_t offset)
{
  CacheEntry entry;
  entry.offset = offset;
  entry.data = std::move(data);

  cache_.push_back(std::move(entry));
}

}  // namespace io
}  // namespace rosbaz
