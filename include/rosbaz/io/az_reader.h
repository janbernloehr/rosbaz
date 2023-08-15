#pragma once

#include "rosbaz/common.h"
#include "rosbaz/internal/nonstd/span.hpp"
#include "rosbaz/io/cache_strategy.h"
#include "rosbaz/io/reader.h"

#include <azure/core/credentials/credentials.hpp>
#include <azure/storage/common/storage_credential.hpp>

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace Azure
{
namespace Storage
{
namespace Blobs
{
class BlobClient;
}
}  // namespace Storage
}  // namespace Azure

namespace rosbaz
{
struct AzBlobUrl;

namespace io
{
class AzReader : public IReader
{
public:
  explicit AzReader(const AzBlobUrl& blob_url, std::unique_ptr<ICacheStrategy> cache_strategy,
                    std::shared_ptr<Azure::Core::Credentials::TokenCredential> credential = nullptr);
  explicit AzReader(const AzBlobUrl& blob_url, std::unique_ptr<ICacheStrategy> cache_strategy,
                    std::shared_ptr<Azure::Storage::StorageSharedKeyCredential> credential);

  explicit AzReader(const AzBlobUrl& blob_url,
                    std::shared_ptr<Azure::Core::Credentials::TokenCredential> credential = nullptr);
  explicit AzReader(const AzBlobUrl& blob_url, std::shared_ptr<Azure::Storage::StorageSharedKeyCredential> credential);

  ~AzReader() override;

  size_t size() override;

  std::string filepath() override;

  void set_cache_hints(const nonstd::span<uint64_t> cache_hints) override;

  const ReaderStatistics& stats() const override;

private:
  void read_fixed(rosbaz::io::byte* buffer, size_t offset, size_t size) override;

  void download(rosbaz::io::byte* buffer, size_t offset, size_t size);

  std::string container_{};
  std::string blob_{};
  std::shared_ptr<Azure::Storage::Blobs::BlobClient> client_{};

  ReaderStatistics stats_{};

  std::unique_ptr<ICacheStrategy> cache_strategy_{ nullptr };
  std::mutex cache_mutex_{};

  std::vector<OffsetAndSize> pending_reads_{};
  std::condition_variable read_completed_;
};

}  // namespace io
}  // namespace rosbaz
