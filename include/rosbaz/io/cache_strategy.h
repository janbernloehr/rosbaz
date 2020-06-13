#pragma once

#include <cstdint>
#include <vector>

#include "rosbaz/common.h"

namespace rosbaz
{
namespace io
{
class ICacheStrategy
{
public:
  virtual bool retrieve(rosbaz::io::byte* buffer, size_t offset, size_t count) = 0;

  virtual void update(std::vector<rosbaz::io::byte> data, size_t offset) = 0;

  virtual size_t cache_element_size(size_t count) = 0;
};

}  // namespace io
}  // namespace rosbaz