#pragma once

#include "rosbaz/common.h"
#include "rosbaz/io/io_helpers.h"
#include "rosbaz/io/reader.h"

#include <cstddef>
#include <memory>
#include <mutex>
#include <string>

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

  const ReaderStatistics& stats() const override;

private:
  void read_fixed(rosbaz::io::byte* buffer, size_t offset, size_t size) override;

  rosbaz::io::RosStream m_source;
  std::string m_filepath;
  std::mutex m_mutex{};

  ReaderStatistics stats_{};
};

}  // namespace io
}  // namespace rosbaz
