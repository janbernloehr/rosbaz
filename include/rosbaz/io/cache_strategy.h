#pragma once

#include "rosbaz/common.h"
#include "rosbaz/internal/nonstd/span.hpp"

#include <cstdint>
#include <cstddef>

namespace rosbaz
{
namespace io
{
class Buffer;

struct OffsetAndSize
{
  size_t offset;
  size_t size;

  size_t begin() const
  {
    return offset;
  }
  size_t end() const
  {
    return offset + size;
  }

  bool contains(size_t offset, size_t size) const;

  friend bool operator==(const OffsetAndSize& lhs, const OffsetAndSize& rhs);
  friend bool operator!=(const OffsetAndSize& lhs, const OffsetAndSize& rhs);
};

class ICacheStrategy
{
public:
  virtual ~ICacheStrategy() = default;

  /// Retrieve a specified range of bytes from the cache.
  ///
  /// \return true if the specified range was found in the cache; otherwise false.
  virtual bool retrieve(rosbaz::io::byte* buffer, size_t offset, size_t size) const = 0;

  /// Update the cache with the specified range of bytes.
  virtual void update(rosbaz::io::Buffer&& data, size_t offset) = 0;

  /// \return The number of bytes to load (returns at least \p size but the cache strategy can decide to request more
  /// data to load).
  virtual OffsetAndSize cache_element_offset_and_size(size_t offset, size_t size) const = 0;

  /// \return true if the caching strategy is valid for the given data; otherwise false.
  virtual bool accepts(size_t offset, size_t size) const = 0;

  /// Use the given offsets as boundary for caching elements.
  virtual void set_cache_hints(const nonstd::span<uint64_t> cache_hints);
};

}  // namespace io
}  // namespace rosbaz
