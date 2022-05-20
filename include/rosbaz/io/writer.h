#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "rosbaz/common.h"
#include "rosbaz/io/io_helpers.h"
#include <boost/optional.hpp>

namespace rosbaz
{
namespace io
{
struct BlockId
{
};

class Block
{
public:
  Block(size_t block_offset);

  virtual ~Block() = default;

  virtual void write(const rosbaz::io::byte* data, size_t n, boost::optional<size_t> offset = boost::none) = 0;

  void write(const std::string& s);

  virtual void stage() = 0;

  bool is_staged() const
  {
    return is_staged_;
  }

  // offset of the block within the bag
  size_t block_offset() const
  {
    return block_offset_;
  };

  virtual size_t size() const = 0;

protected:
  bool is_staged_{ false };
  size_t block_offset_{ 0 };
};

class IWriter
{
public:
  virtual ~IWriter() = default;

  virtual size_t size() = 0;

  virtual std::string filepath() = 0;

  virtual std::shared_ptr<Block> create_block() = 0;

  virtual std::shared_ptr<Block> replace_block(Block& original) = 0;

  virtual void commit_blocks() = 0;

  virtual bool has_unstaged_blocks() const = 0;
};

}  // namespace io
}  // namespace rosbaz