#include "rosbaz/io/writer.h"

#include "rosbaz/io/io_helpers.h"

namespace rosbaz
{
namespace io
{
Block::Block(size_t block_offset) : block_offset_{ block_offset }
{
}

void Block::write(const std::string& s)
{
  write(reinterpret_cast<const rosbaz::io::byte*>(s.c_str()), s.length());
}

}  // namespace io
}  // namespace rosbaz
