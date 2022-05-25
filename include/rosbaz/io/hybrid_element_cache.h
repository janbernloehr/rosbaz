#pragma once

#include <cstdint>

#include <boost/circular_buffer.hpp>

#include "rosbaz/io/cache_entry.h"
#include "rosbaz/io/cache_strategy.h"

namespace rosbaz
{
namespace io
{
/// Like \p SmallElementCacheStrategy but also caches large chunks.
///
/// With default parameters, HybridElementCacheStrategy caches up to 10k small chunks of size 1k and hence requires 10M
/// of memory for the small chunk cache. In addition, the last 100 large chunks are cached and the memory required
/// depends on the size of those chunks.
class HybridElementCacheStrategy : public ICacheStrategy
{
public:
  HybridElementCacheStrategy(size_t max_small_element_size = 1024, size_t max_small_elements = 10000,
                             size_t max_large_elements = 100);

  bool retrieve(rosbaz::io::byte* buffer, size_t offset, size_t size) const override;

  void update(rosbaz::io::Buffer&& data, size_t offset) override;

  OffsetAndSize cache_element_offset_and_size(size_t offset, size_t size) const override;

  bool accepts(size_t offset, size_t size) const override;

private:
  const size_t max_small_element_size_;
  const size_t max_small_elements_;
  const size_t max_large_elements_;

  boost::circular_buffer<CacheEntry> small_element_cache_{ max_small_elements_ };
  boost::circular_buffer<CacheEntry> large_element_cache_{ max_large_elements_ };
};
}  // namespace io
}  // namespace rosbaz
