#pragma once

#include "rosbaz/common.h"
#include "rosbaz/io/cache_entry.h"
#include "rosbaz/io/cache_strategy.h"

#include <boost/circular_buffer.hpp>
#include <cstddef>

namespace rosbaz
{
namespace io
{
class Buffer;

/// This cache strategy aims at reducing the number of reads towards the storage for small queries.
///
/// (1) in the common case, the cache holds blocks of \p element_size
/// (2) larger requests bypass the cache
/// (3) all smaller requests are rounded up to \p element_size
/// (4) the actual stored data might be less than \p element_size for the block at the end of the file
///
/// With default parameters, SmallElementCacheStrategy caches up to 10k small chunks of size 1k and hence requires 10M
/// of memory.
class SmallElementCacheStrategy : public ICacheStrategy
{
public:
  SmallElementCacheStrategy(size_t element_size = 1024, size_t max_elements = 10000);

  bool retrieve(rosbaz::io::byte* buffer, size_t offset, size_t size) const override;

  void update(rosbaz::io::Buffer&& data, size_t offset) override;

  OffsetAndSize cache_element_offset_and_size(size_t offset, size_t size) const override;

  bool accepts(size_t offset, size_t size) const override;

private:
  const size_t element_size_;
  const size_t max_elements_;

  boost::circular_buffer<CacheEntry> cache_{ max_elements_ };
};

}  // namespace io
}  // namespace rosbaz
