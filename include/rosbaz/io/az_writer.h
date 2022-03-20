#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

#include "rosbaz/blob_url.h"
#include "rosbaz/io/writer.h"

namespace Azure
{
namespace Storage
{
namespace Blobs
{
class BlockBlobClient;
}
}  // namespace Storage
}  // namespace Azure

namespace rosbaz
{
namespace io
{
class AzWriter;

class AzBlock : public Block
{
public:
  AzBlock(AzWriter& writer, size_t block_offset);

  virtual ~AzBlock() = default;

  void write(rosbaz::io::byte const* data, size_t n, boost::optional<size_t> offset = boost::none) override;

  size_t size() const override;

  void stage() override;

  const std::string& id() const { return id_; }

private:
  AzWriter& writer_;

  std::vector<rosbaz::io::byte> buffer_{};

  std::string id_;
};


class AzWriter : public IWriter
{
public:
  explicit AzWriter(const AzBlobUrl& blob_url, const std::string& account_key = "", const std::string& token = "");

  ~AzWriter() override;

  size_t size() override;

  std::string filepath() override;

  std::shared_ptr<Block> create_block() override;

  std::shared_ptr<Block> replace_block(Block& original) override;

  void commit_blocks() override;

  bool has_unstaged_blocks() const override;

  std::int32_t num_requests() const
  {
    return num_requests_;
  }
  std::int64_t num_bytes() const
  {
    return num_bytes_;
  }

private:
  std::string container_{};
  std::string blob_{};
  std::shared_ptr<Azure::Storage::Blobs::BlockBlobClient> client_{};

  std::int32_t num_requests_{ 0 };
  std::int64_t num_bytes_{ 0 };

  std::vector<std::shared_ptr<AzBlock>> blocks_{};

  std::mutex mutex_{};

  friend AzBlock;
};

}  // namespace io
}  // namespace rosbaz