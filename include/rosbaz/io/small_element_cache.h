#pragma once

#include <cstdint>

#include "rosbaz/io/cache_strategy.h"

namespace rosbaz {

namespace io {

struct CacheEntry {
  std::vector<byte> data;
  std::uint64_t offset;
};

class SmallElementCacheStrategy : public ICacheStrategy {
 public:
  SmallElementCacheStrategy(size_t max_element_size = 1024,
                            size_t max_elements = 10000);

  bool retrieve(rosbaz::io::byte *buffer, size_t offset, size_t count) override;

  void update(std::vector<rosbaz::io::byte> data, size_t offset) override;

  size_t cache_element_size(size_t count) override;

 private:
  std::vector<CacheEntry> cache_{};
  const size_t max_element_size_;
  const size_t max_elements_;
};
}  // namespace io
}  // namespace rosbaz