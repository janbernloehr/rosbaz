#include "rosbaz/io/reader.h"

#include "rosbaz/bag_parsing/record.h"
#include "rosbaz/io/io_helpers.h"

#include <algorithm>

namespace rosbaz
{
namespace io
{
HeaderBufferAndSize IReader::read_header_buffer_and_size(size_t offset)
{
  HeaderBufferAndSize result;
  result.header_size = read_little_endian<uint32_t>(offset);

  result.header_buffer = read(offset + sizeof(uint32_t), result.header_size + sizeof(uint32_t));

  result.data_size = rosbaz::io::read_little_endian<uint32_t>(result.header_buffer, result.header_size);
  result.header_buffer.resize(result.header_size);

  return result;
}

HeaderBufferAndSize parseHeaderBufferAndSize(const rosbaz::DataSpan buffer_span)
{
  HeaderBufferAndSize result;

  const auto record = rosbaz::bag_parsing::Record::parse(buffer_span);

  result.header_size = record.header_length;

  result.header_buffer.resize(result.header_size);
  std::copy(record.header.begin(), record.header.end(), result.header_buffer.data());

  result.data_size = record.data_length;

  return result;
}

void IReader::set_cache_hints(const nonstd::span<uint64_t>)
{
}

rosbaz::io::Buffer IReader::read(size_t offset, size_t size)
{
  rosbaz::io::Buffer buffer(size);
  read_fixed(buffer.data(), offset, size);
  return buffer;
}

}  // namespace io
}  // namespace rosbaz
