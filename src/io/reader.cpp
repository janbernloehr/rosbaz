#include "rosbaz/io/reader.h"

#include "rosbaz/io/io_helpers.h"

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

rosbaz::io::Buffer IReader::read(size_t offset, size_t count)
{
  rosbaz::io::Buffer buffer(count);
  read_fixed(buffer.data(), offset, count);
  return buffer;
}

}  // namespace io
}  // namespace rosbaz
