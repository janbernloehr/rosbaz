#pragma once

#include <cstdint>
#include <vector>

#include <boost/circular_buffer.hpp>

#include "rosbaz/io/cache_entry.h"
#include "rosbaz/io/cache_strategy.h"

namespace rosbaz
{
namespace io
{
/// This cache strategy aims at reducing the number of reads towards the storage for all queries.
///
/// (1) in the common case, the cache holds blocks of approximate size \p element_size
/// (2) if cache hints are present, requests of [a,b] are enlarged to the smallest interval [a',b'] so that
///     a' and b' are cache hints and b'-a' >= \p element_size
/// (2) if cache hints are not present, requests are just rounded up to \p element_size
/// (4) the actual stored data might be less than \p element_size for the block at the end of the file
///
/// With default parameters, ChunkInformedCacheStrategy requires approximately 2GB of memory.
class ChunkInformedCacheStrategy : public ICacheStrategy
{
public:
  ChunkInformedCacheStrategy(size_t element_size = 4 * 1024 * 1024, size_t max_elements = 500);

  bool retrieve(rosbaz::io::byte* buffer, size_t offset, size_t size) const override;

  void update(rosbaz::io::Buffer&& data, size_t offset) override;

  OffsetAndSize cache_element_offset_and_size(size_t offset, size_t size) const override;

  bool accepts(size_t offset, size_t size) const override;

  void set_cache_hints(const nonstd::span<uint64_t> cache_hints) override;

private:
  const size_t element_size_;
  const size_t max_elements_;

  boost::circular_buffer<CacheEntry> cache_{ max_elements_ };

  std::vector<uint64_t> cache_hints_{};
};

}  // namespace io
}  // namespace rosbaz
