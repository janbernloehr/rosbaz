#pragma once

#include <cstdint>

#include <boost/circular_buffer.hpp>

#include "rosbaz/io/cache_entry.h"
#include "rosbaz/io/cache_strategy.h"

namespace rosbaz
{
namespace io
{
/// Caches the most recent small chunks of a bag file. The reader reads at least \p max_element_size_ bytes even when
/// fewer are requested. Can significantly reduce the number of reads when indexing a bag and when retrieving small
/// chunks.
///
/// With default parameters, SmallElementCacheStrategy caches up to 10k small chunks of size 1k and hence requires 10M
/// of memory.
class SmallElementCacheStrategy : public ICacheStrategy
{
public:
  SmallElementCacheStrategy(size_t max_element_size = 1024, size_t max_elements = 10000);

  bool retrieve(rosbaz::io::byte* buffer, size_t offset, size_t count) override;

  void update(std::vector<rosbaz::io::byte> data, size_t offset) override;

  size_t cache_element_size(size_t count) override;

private:
  const size_t max_element_size_;
  const size_t max_elements_;

  boost::circular_buffer<CacheEntry> cache_{ max_elements_ };
};

}  // namespace io
}  // namespace rosbaz