#include "rosbaz/io/stream_reader.h"

#include "rosbaz/exceptions.h"
#include "rosbaz/io/io_helpers.h"
#include "rosbaz/io/util.h"

namespace rosbaz
{
namespace io
{
std::unique_ptr<IReader> StreamReader::open(const std::string& file_path)
{
  rosbaz::io::RosStream ifs;
  ifs.open(file_path, std::ios_base::in | std::ios_base::binary);

  if (!ifs.good())
  {
    throw rosbaz::IoException("Error opening file: " + file_path);
  }

  return std::make_unique<rosbaz::io::StreamReader>(std::move(ifs), file_path);
}

StreamReader::StreamReader(rosbaz::io::RosStream source, const std::string& filepath)
  : m_source(std::move(source)), m_filepath(filepath)
{
}

size_t StreamReader::size()
{
  std::lock_guard<std::mutex> lock_guard(m_mutex);
  m_source.seekg(0, rosbaz::io::RosStream::end);
  return rosbaz::io::narrow<size_t>(m_source.tellg());
}

std::string StreamReader::filepath()
{
  return m_filepath;
}

void StreamReader::read_fixed(rosbaz::io::byte* buffer, size_t offset, size_t size)
{
  std::lock_guard<std::mutex> lock_guard(m_mutex);
  m_source.seekg(rosbaz::io::narrow<std::streampos>(offset));
  read_from(m_source, buffer, size);
}

const ReaderStatistics& StreamReader::stats() const
{
  return stats_;
}

}  // namespace io
}  // namespace rosbaz
