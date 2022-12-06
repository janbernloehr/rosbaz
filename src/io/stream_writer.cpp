#include "rosbaz/io/stream_writer.h"

#include "rosbaz/exceptions.h"
#include "rosbaz/io/util.h"

#include <algorithm>
#include <numeric>
#include <ostream>
#include <utility>

namespace rosbaz
{
namespace io
{
StreamBlock::StreamBlock(StreamWriter& writer)
  : Block{ rosbaz::io::narrow<size_t>(writer.m_target.tellp()) }, writer_{ writer }
{
}

void StreamBlock::write(rosbaz::io::byte const* data, size_t n, boost::optional<size_t> offset)
{
  if (is_staged())
  {
    throw BlockStagedException("Cannot write to staged block");
  }
  if (offset)
  {
    writer_.m_target.seekp(rosbaz::io::narrow<std::streampos>(block_offset_ + *offset));
  }
  writer_.m_target.write(reinterpret_cast<const char*>(data), n);

  if (offset)
  {
    size_ = std::max(size_, *offset + n);
  }
  else
  {
    size_ += n;
  }
}

size_t StreamBlock::size() const
{
  return size_;
}

void StreamBlock::stage()
{
  is_staged_ = true;
  writer_.m_target.seekp(rosbaz::io::narrow<std::streampos>(block_offset_ + size()));
}

std::unique_ptr<IWriter> StreamWriter::open(const std::string& file_path)
{
  std::ofstream ofs;
  ofs.open(file_path, std::ios_base::out | std::ios_base::binary);

  if (!ofs.good())
  {
    throw rosbaz::IoException("Error opening file: " + file_path);
  }

  return std::unique_ptr<rosbaz::io::IWriter>{ new rosbaz::io::StreamWriter{ std::move(ofs), file_path } };
}

StreamWriter::StreamWriter(std::ofstream target, const std::string& filepath)
  : m_target(std::move(target)), m_filepath(filepath)
{
}

size_t StreamWriter::size()
{
  std::lock_guard<std::mutex> lock_guard(m_mutex);

  return std::accumulate(m_blocks.begin(), m_blocks.end(), 0,
                         [](size_t a, const auto& block) { return block->is_staged() ? a + block->size() : a; });
}

std::string StreamWriter::filepath()
{
  return m_filepath;
}

std::shared_ptr<Block> StreamWriter::create_block()
{
  std::lock_guard<std::mutex> lock_guard(m_mutex);
  if (has_unstaged_blocks())
  {
    throw UnstagedBlocksException("Cannot create a new block while unstaged blocks present");
  }

  auto block = std::make_shared<StreamBlock>(*this);
  m_blocks.push_back(block);
  return block;
}

std::shared_ptr<Block> StreamWriter::replace_block(Block& original)
{
  m_target.seekp(rosbaz::io::narrow<std::streampos>(original.block_offset()));
  // TODO: remove id
  return create_block();
}

void StreamWriter::commit_blocks()
{
  // noop
}

bool StreamWriter::has_unstaged_blocks() const
{
  return std::any_of(m_blocks.begin(), m_blocks.end(), [](const auto& block) { return !block->is_staged(); });
}

StreamWriter::~StreamWriter()
{
  m_target.close();
}

}  // namespace io
}  // namespace rosbaz
