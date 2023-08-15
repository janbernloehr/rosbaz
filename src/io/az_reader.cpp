#include "rosbaz/io/az_reader.h"

#ifndef NO_AZ_BINDINGS

#include "rosbaz/blob_url.h"
#include "rosbaz/exceptions.h"
#include "rosbaz/io/buffer.h"
#include "rosbaz/io/chunk_informed_cache.h"

#include <azure/core.hpp>
#include <azure/storage/blobs.hpp>
#include <boost/make_unique.hpp>

#include <algorithm>
#include <cassert>
#include <regex>
#include <sstream>
#include <utility>

namespace rosbaz
{
namespace io
{
AzReader::AzReader(const AzBlobUrl& blob_url, std::shared_ptr<Azure::Core::Credentials::TokenCredential> credential)
  : AzReader(blob_url, boost::make_unique<rosbaz::io::ChunkInformedCacheStrategy>(), credential)
{
}
AzReader::AzReader(const AzBlobUrl& blob_url, std::shared_ptr<Azure::Storage::StorageSharedKeyCredential> credential)
  : AzReader(blob_url, boost::make_unique<rosbaz::io::ChunkInformedCacheStrategy>(), credential)
{
}

AzReader::AzReader(const AzBlobUrl& blob_url, std::unique_ptr<ICacheStrategy> cache_strategy,
                   std::shared_ptr<Azure::Core::Credentials::TokenCredential> credential)
  : container_(blob_url.container_name), blob_(blob_url.blob_name), cache_strategy_(std::move(cache_strategy))
{
  if (credential != nullptr)
  {
    client_ = std::make_shared<Azure::Storage::Blobs::BlobClient>(blob_url.to_string(), credential);
  }
  else
  {
    client_ = std::make_shared<Azure::Storage::Blobs::BlobClient>(blob_url.to_string());
  }
}

AzReader::AzReader(const AzBlobUrl& blob_url, std::unique_ptr<ICacheStrategy> cache_strategy,
                   std::shared_ptr<Azure::Storage::StorageSharedKeyCredential> credential)
  : container_(blob_url.container_name), blob_(blob_url.blob_name), cache_strategy_(std::move(cache_strategy))
{
  client_ = std::make_shared<Azure::Storage::Blobs::BlobClient>(blob_url.to_string(), credential);
}

AzReader::~AzReader() = default;

std::string AzReader::filepath()
{
  std::regex signature_pattern("sig=([^&]+)");
  return std::regex_replace(client_->GetUrl(), signature_pattern, "sig=REDACTED");
}

size_t AzReader::size()
{
  auto ret = client_->GetProperties();

  stats_.num_reads += 1;

  return static_cast<uint64_t>(ret.Value.BlobSize);
}

void AzReader::set_cache_hints(const nonstd::span<uint64_t> cache_hints)
{
  if (cache_strategy_ == nullptr)
  {
    return;
  }

  cache_strategy_->set_cache_hints(cache_hints);
}

void AzReader::download(rosbaz::io::byte* buffer, size_t offset, size_t size)
{
  Azure::Storage::Blobs::DownloadBlobToOptions options{};
  options.Range = Azure::Core::Http::HttpRange{};
  options.Range.Value().Offset = static_cast<int64_t>(offset);
  options.Range.Value().Length = static_cast<int64_t>(size);

  client_->DownloadTo(buffer, size, options);

  stats_.num_reads += 1;
  stats_.num_bytes_read += size;
}

void AzReader::read_fixed(rosbaz::io::byte* buffer, size_t offset, size_t size)
{
  if ((cache_strategy_ == nullptr) || (!cache_strategy_->accepts(offset, size)))
  {
    download(buffer, offset, size);
    return;
  }

  OffsetAndSize download_offset_and_size;
  {
    std::unique_lock<std::mutex> lock(cache_mutex_);

    if (cache_strategy_->retrieve(buffer, offset, size))
    {
      return;
    }

    read_completed_.wait(lock, [this, offset, size]() {
      auto found_pending_read =
          std::find_if(pending_reads_.begin(), pending_reads_.end(),
                       [offset, size](const auto& pending_read) { return pending_read.contains(offset, size); });

      return found_pending_read == pending_reads_.end();
    });

    if (cache_strategy_->retrieve(buffer, offset, size))
    {
      return;
    }

    download_offset_and_size = cache_strategy_->cache_element_offset_and_size(offset, size);
    assert(download_offset_and_size.offset <= offset);
    assert(download_offset_and_size.offset + download_offset_and_size.size >= offset + size);
    pending_reads_.push_back(download_offset_and_size);
  }

  rosbaz::io::Buffer download_buffer{ download_offset_and_size.size };

  download(download_buffer.data(), download_offset_and_size.offset, download_offset_and_size.size);
  assert(offset >= download_offset_and_size.offset);
  const size_t buffer_offset = offset - download_offset_and_size.offset;
  assert(buffer_offset + size <= download_buffer.size());
  std::copy_n(download_buffer.begin() + buffer_offset, size, buffer);

  {
    std::lock_guard<std::mutex> lock_guard(cache_mutex_);

    cache_strategy_->update(std::move(download_buffer), download_offset_and_size.offset);

    pending_reads_.erase(std::remove_if(pending_reads_.begin(), pending_reads_.end(),
                                        [&download_offset_and_size](const auto& pending_read) {
                                          return pending_read == download_offset_and_size;
                                        }),
                         pending_reads_.end());
  }

  read_completed_.notify_all();
}

const ReaderStatistics& AzReader::stats() const
{
  return stats_;
}

}  // namespace io
}  // namespace rosbaz
#endif
