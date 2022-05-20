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
/// Caches the most recent message chunks of a bag file. Message chunks are identified using the provided
/// cache hints so that the reader reads at least \p max_element_size bytes from one cache hint to the net. Can
/// significantly reduce the number of reads when reading many small messages from a large chunk.
class ChunkInformedCacheStrategy : public ICacheStrategy
{
public:
  ChunkInformedCacheStrategy(size_t max_element_size = 4 * 1024 * 1024, size_t max_elements = 500);

  bool retrieve(rosbaz::io::byte* buffer, size_t offset, size_t count) override;

  void update(rosbaz::io::Buffer&& data, size_t offset) override;

  OffsetAndSize cache_element_offset_and_size(size_t offset, size_t count) const override;

  bool accepts(size_t offset, size_t count) const override;

  void use_cache_hints(const std::vector<uint64_t>& cache_hints) override;

private:
  const size_t max_element_size_;
  const size_t max_elements_;

  boost::circular_buffer<CacheEntry> cache_{ max_elements_ };

  std::vector<uint64_t> cache_hints_{};
};

}  // namespace io
}  // namespace rosbaz
