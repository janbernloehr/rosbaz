#include "rosbaz/io/chunk_informed_cache.h"

#include <sstream>
#include <iostream>
namespace rosbaz
{
namespace io
{
ChunkInformedCacheStrategy::ChunkInformedCacheStrategy(const size_t element_size, const size_t max_elements,
                                                       const size_t index_element_size)
  : element_size_(element_size), max_elements_(max_elements), index_element_size_(index_element_size)
{
}

bool ChunkInformedCacheStrategy::retrieve(rosbaz::io::byte* buffer, size_t offset, size_t size) const
{
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

void ChunkInformedCacheStrategy::set_cache_hints(const nonstd::span<uint64_t> cache_hints)
{
  cache_hints_.resize(cache_hints.size());
  std::copy(cache_hints.begin(), cache_hints.end(), cache_hints_.begin());
}

OffsetAndSize ChunkInformedCacheStrategy::cache_element_offset_and_size(size_t offset, size_t size) const
{
  if (cache_hints_.empty())
  {
    return OffsetAndSize{ offset, std::max(index_element_size_, size) };
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

    auto end_it = std::lower_bound(cache_hints_.begin(), cache_hints_.end(), offset + std::max(element_size_, size));
    size_t end = cache_hints_.back();
    if (end_it != cache_hints_.end())
    {
      end = *end_it;

      if ((end - begin) > 2 * element_size_)
      {
        --end_it;
        if (*end_it >= offset + size)
        {
          end = *end_it;
        }
      }
    }

    return OffsetAndSize{ begin, end - begin };
  }
}

bool ChunkInformedCacheStrategy::accepts(size_t /* offset */, size_t /* size */) const
{
  return true;
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
