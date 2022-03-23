#pragma once

#include <cstdint>
#include <vector>

#include "rosbaz/common.h"
#include "rosbaz/io/buffer.h"

namespace rosbaz
{
namespace io
{
class ICacheStrategy
{
public:
  virtual ~ICacheStrategy() = default;

  /// Retrieve a specified range of bytes from the cache.
  ///
  /// \return true if the specified range was found in the cache; otherwise false.
  virtual bool retrieve(rosbaz::io::byte* buffer, size_t offset, size_t count) = 0;

  /// Update the cache with the specified range of bytes.
  virtual void update(rosbaz::io::Buffer&& data, size_t offset) = 0;

  /// \return The number of bytes to load (returns at least \p count but the cache strategy can decide to request more
  /// data to load).
  virtual size_t cache_element_size(size_t count) = 0;
};

}  // namespace io
}  // namespace rosbaz