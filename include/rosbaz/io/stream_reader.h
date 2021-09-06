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
  explicit StreamReader(rosbaz::io::RosStream source, const std::string& filepath = "");

  size_t size() override;

  std::string filepath() override;

  static std::unique_ptr<IReader> open(const std::string& file_path);

private:
  void read_fixed(rosbaz::io::byte* buffer, size_t offset, size_t count) override;

  rosbaz::io::RosStream m_source;
  std::string m_filepath;
  std::mutex m_mutex{};
};

}  // namespace io
}  // namespace rosbaz