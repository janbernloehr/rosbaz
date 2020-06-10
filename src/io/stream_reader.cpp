#include "rosbaz/io/stream_reader.h"

#include "rosbaz/io/io_helpers.h"

namespace rosbaz {

namespace io {

std::unique_ptr<IReader> StreamReader::open(const std::string &file_path) {
  rosbaz::io::RosStream ifs;
  ifs.open(file_path, std::ios_base::in | std::ios_base::binary);

  return std::unique_ptr<rosbaz::io::IReader>{
      new rosbaz::io::StreamReader{std::move(ifs)}};
}

StreamReader::StreamReader(rosbaz::io::RosStream source)
    : m_source(std::move(source)) {}

size_t StreamReader::size() {
  m_source.seekg(0, rosbaz::io::RosStream::end);
  return m_source.tellg();
}

void StreamReader::read_fixed(rosbaz::io::byte *buffer, size_t offset,
                              size_t count) {
  m_source.seekg(offset);
  read_from(m_source, buffer, count);
}

} // namespace io
} // namespace rosbaz