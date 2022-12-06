#pragma once

#include "rosbaz/common.h"
#include "rosbaz/internal/nonstd/span.hpp"
#include "rosbaz/io/cache_entry.h"
#include "rosbaz/io/cache_strategy.h"

#include <boost/circular_buffer.hpp>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace rosbaz
{
namespace io
{
class Buffer;

/// This cache strategy aims at reducing the number of reads towards the storage for all queries.
///
/// (1) in the common case, the cache holds blocks of approximate size \p element_size
/// (2) if cache hints are present, requests of [a,b] are enlarged to the smallest interval [a',b'] so that
///     a' and b' are cache hints and b'-a' >= \p element_size
/// (2) if cache hints are not present, requests are just rounded up to \p element_size
/// (4) the actual stored data might be less than \p element_size for the block at the end of the file
///
/// With default parameters, ChunkInformedCacheStrategy requires approximately 64MB of memory.
class ChunkInformedCacheStrategy : public ICacheStrategy
{
public:
  ChunkInformedCacheStrategy(size_t element_size = 64 * 1024, size_t max_elements = 1000,
                             size_t index_element_size = 2 * 1024);

  bool retrieve(rosbaz::io::byte* buffer, size_t offset, size_t size) const override;

  void update(rosbaz::io::Buffer&& data, size_t offset) override;

  OffsetAndSize cache_element_offset_and_size(size_t offset, size_t size) const override;

  bool accepts(size_t offset, size_t size) const override;

  void set_cache_hints(const nonstd::span<uint64_t> cache_hints) override;

private:
  const size_t element_size_;
  const size_t max_elements_;
  const size_t index_element_size_;

  boost::circular_buffer<CacheEntry> cache_{ max_elements_ };

  std::vector<uint64_t> cache_hints_{};
};

}  // namespace io
}  // namespace rosbaz
