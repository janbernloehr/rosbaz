#pragma once

#include "rosbaz/common.h"
#include "rosbaz/io/writer.h"

#include <boost/optional.hpp>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <azure/core/credentials/credentials.hpp>
#include <azure/storage/common/storage_credential.hpp>

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
struct AzBlobUrl;

namespace io
{
class AzWriter;
class Buffer;

class AzBlock : public Block
{
public:
  AzBlock(AzWriter& writer, size_t block_offset);

  virtual ~AzBlock() = default;

  void write(rosbaz::io::byte const* data, size_t n, boost::optional<size_t> offset = boost::none) override;

  size_t size() const override;

  void stage() override;

  const std::string& id() const
  {
    return id_;
  }

private:
  AzWriter& writer_;

  std::shared_ptr<rosbaz::io::Buffer> buffer_{};

  std::string id_;

  size_t block_size{ 0 };
};

class AzWriter : public IWriter
{
public:
  explicit AzWriter(const AzBlobUrl& blob_url,
                    std::shared_ptr<Azure::Core::Credentials::TokenCredential> credential = nullptr);
  explicit AzWriter(const AzBlobUrl& blob_url, std::shared_ptr<Azure::Storage::StorageSharedKeyCredential> credential);

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
