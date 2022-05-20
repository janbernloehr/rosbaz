#pragma once

#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

#include "rosbaz/blob_url.h"
#include "rosbaz/io/cache_strategy.h"
#include "rosbaz/io/reader.h"

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
namespace io
{
class AzReader : public IReader
{
public:
  explicit AzReader(const AzBlobUrl& blob_url, const std::string& account_key = "", const std::string& token = "");

  explicit AzReader(const AzBlobUrl& blob_url, std::unique_ptr<ICacheStrategy> cache_strategy);

  explicit AzReader(const AzBlobUrl& blob_url, const std::string& account_key, const std::string& token,
                    std::unique_ptr<ICacheStrategy> cache_strategy);

  ~AzReader() override;

  size_t size() override;

  std::string filepath() override;

  std::int32_t num_requests() const
  {
    return num_requests_;
  }
  std::int64_t num_bytes() const
  {
    return num_bytes_;
  }

  void use_cache_hints(const std::vector<uint64_t>& cache_hints) override;

private:
  void read_fixed(rosbaz::io::byte* buffer, size_t offset, size_t count) override;

  void download(rosbaz::io::byte* buffer, size_t offset, size_t count);

  std::string container_{};
  std::string blob_{};
  std::shared_ptr<Azure::Storage::Blobs::BlobClient> client_{};

  std::int32_t num_requests_{ 0 };
  std::int64_t num_bytes_{ 0 };

  std::unique_ptr<ICacheStrategy> cache_strategy_{ nullptr };
  std::mutex cache_mutex_{};

  std::vector<OffsetAndSize> pending_reads_{};
  std::condition_variable read_available_;
};

}  // namespace io
}  // namespace rosbaz