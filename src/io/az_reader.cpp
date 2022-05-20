#include "rosbaz/io/az_reader.h"

#ifndef NO_AZ_BINDINGS

#include "az_bearer_token.h"

#include <ros/console.h>

#include <azure/core.hpp>
#include <azure/storage/blobs.hpp>
#include <boost/make_unique.hpp>

#include "rosbaz/exceptions.h"
#include "rosbaz/io/io_helpers.h"
#include "rosbaz/io/chunk_informed_cache.h"

namespace rosbaz
{
namespace io
{
AzReader::AzReader(const AzBlobUrl& blob_url, const std::string& account_key, const std::string& token)
  : AzReader(blob_url, account_key, token, boost::make_unique<rosbaz::io::ChunkInformedCacheStrategy>())
{
}

AzReader::AzReader(const AzBlobUrl& blob_url, std::unique_ptr<ICacheStrategy> cache_strategy)
  : AzReader(blob_url, "", "", std::move(cache_strategy))
{
}

AzReader::AzReader(const AzBlobUrl& blob_url, const std::string& account_key, const std::string& token,
                   std::unique_ptr<ICacheStrategy> cache_strategy)
  : container_(blob_url.container_name), blob_(blob_url.blob_name), cache_strategy_(std::move(cache_strategy))
{
  std::shared_ptr<Azure::Storage::Blobs::BlobClient> blobClient;

  if (!token.empty())
  {
    auto credential = std::make_shared<BearerToken>(token);
    client_ = std::make_shared<Azure::Storage::Blobs::BlobClient>(blob_url.to_string(), credential);
  }
  else if (!blob_url.sas_token.empty())
  {
    client_ = std::make_shared<Azure::Storage::Blobs::BlobClient>(blob_url.to_string());
  }
  else if (!account_key.empty())
  {
    auto credential = std::make_shared<Azure::Storage::StorageSharedKeyCredential>(blob_url.account_name, account_key);
    client_ = std::make_shared<Azure::Storage::Blobs::BlobClient>(blob_url.to_string(), credential);
  }
  else
  {
    throw rosbaz::MissingCredentialsException("You must provide either a bearer token, a sas "
                                              "token, or an account key.");
  }
}

AzReader::~AzReader() = default;

std::string AzReader::filepath()
{
  return client_->GetUrl();
}

size_t AzReader::size()
{
  auto ret = client_->GetProperties();

  num_requests_ += 1;

  return ret.Value.BlobSize;
}

void AzReader::use_cache_hints(const std::vector<uint64_t>& cache_hints)
{
  if (cache_strategy_ == nullptr)
  {
    return;
  }

  cache_strategy_->use_cache_hints(cache_hints);
}

void AzReader::download(rosbaz::io::byte* buffer, size_t offset, size_t count)
{
  Azure::Storage::Blobs::DownloadBlobToOptions options{};
  options.Range = Azure::Core::Http::HttpRange{};
  options.Range.Value().Offset = offset;
  options.Range.Value().Length = count;

  client_->DownloadTo(buffer, count, options);

  num_bytes_ += count;
  num_requests_ += 1;
}

void AzReader::read_fixed(rosbaz::io::byte* buffer, size_t offset, size_t count)
{
  if ((cache_strategy_ == nullptr) || (!cache_strategy_->accepts(offset, count)))
  {
    download(buffer, offset, count);
    return;
  }

  OffsetAndSize download_offset_and_size;
  {
    std::unique_lock<std::mutex> lock(cache_mutex_);

    if (cache_strategy_->retrieve(buffer, offset, count))
    {
      return;
    }

    read_available_.wait(lock, [this, offset, count]() {
      auto found_pending_read =
          std::find_if(pending_reads_.begin(), pending_reads_.end(),
                       [offset, count](const auto& pending_read) { return pending_read.contains(offset, count); });

      return found_pending_read == pending_reads_.end();
    });

    if (cache_strategy_->retrieve(buffer, offset, count))
    {
      return;
    }

    download_offset_and_size = cache_strategy_->cache_element_offset_and_size(offset, count);
    pending_reads_.push_back(download_offset_and_size);
  }

  rosbaz::io::Buffer download_buffer{ download_offset_and_size.size };

  download(download_buffer.data(), download_offset_and_size.offset, download_offset_and_size.size);
  std::copy_n(download_buffer.begin() + offset - download_offset_and_size.offset, count, buffer);

  {
    std::lock_guard<std::mutex> lock_guard(cache_mutex_);

    cache_strategy_->update(std::move(download_buffer), download_offset_and_size.offset);

    pending_reads_.erase(std::remove_if(pending_reads_.begin(), pending_reads_.end(),
                                        [&download_offset_and_size](const auto& pending_read) {
                                          return pending_read == download_offset_and_size;
                                        }),
                         pending_reads_.end());
  }

  read_available_.notify_all();
}

}  // namespace io
}  // namespace rosbaz
#endif
