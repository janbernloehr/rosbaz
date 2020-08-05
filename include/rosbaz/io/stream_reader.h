#pragma once

#include <cstdint>
#include <memory>
#include <vector>
#include <mutex>

#include "rosbaz/io/reader.h"

namespace rosbaz
{
namespace io
{
class StreamReader : public IReader
{
public:
  explicit StreamReader(rosbaz::io::RosStream source);

  size_t size() override;

  static std::unique_ptr<IReader> open(const std::string& file_path);

private:
  void read_fixed(rosbaz::io::byte* buffer, size_t offset, size_t count) override;

  rosbaz::io::RosStream m_source;
  std::mutex m_mutex{};
};

}  // namespace io
}  // namespace rosbaz