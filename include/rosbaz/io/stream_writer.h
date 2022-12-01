#pragma once

#include "rosbaz/common.h"
#include "rosbaz/io/writer.h"

#include <boost/optional.hpp>
#include <cstddef>
#include <fstream>
#include <iosfwd>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace rosbaz
{
namespace io
{
class StreamWriter;

class StreamBlock : public Block
{
public:
  StreamBlock(StreamWriter& writer);

  virtual ~StreamBlock() = default;

  void write(rosbaz::io::byte const* data, size_t n, boost::optional<size_t> offset = boost::none) override;

  size_t size() const override;

  void stage() override;

private:
  StreamWriter& writer_;
  size_t size_{ 0 };
};

class StreamWriter : public IWriter
{
public:
  explicit StreamWriter(std::ofstream target, const std::string& filepath = "");

  ~StreamWriter();

  size_t size() override;

  std::string filepath() override;

  static std::unique_ptr<IWriter> open(const std::string& file_path);

  std::shared_ptr<Block> create_block() override;

  std::shared_ptr<Block> replace_block(Block& original) override;

  void commit_blocks() override;

  bool has_unstaged_blocks() const override;

private:
  std::ofstream m_target;
  std::string m_filepath;
  std::mutex m_mutex{};

  std::vector<std::shared_ptr<StreamBlock>> m_blocks{};

  friend StreamBlock;
};

}  // namespace io
}  // namespace rosbaz