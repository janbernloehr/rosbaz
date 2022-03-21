#include "rosbaz/io/az_reader.h"

#include "az_bearer_token.h"

#include <ros/console.h>

#include <azure/core.hpp>
#include <azure/storage/blobs.hpp>
#include <boost/make_unique.hpp>

#include "rosbaz/exceptions.h"
#include "rosbaz/io/io_helpers.h"
#include "rosbaz/io/small_element_cache.h"


namespace rosbaz
{
namespace io
{
AzReader::AzReader(const AzBlobUrl& blob_url, const std::string& account_key, const std::string& token)
  : AzReader(blob_url, account_key, token, boost::make_unique<rosbaz::io::SmallElementCacheStrategy>())
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
  if (cache_strategy_ == nullptr)
  {
    download(buffer, offset, count);
    return;
  }

  {
    std::lock_guard<std::mutex> lock_guard(cache_mutex_);

    if (cache_strategy_->retrieve(buffer, offset, count))
    {
      return;
    }
  }

  const size_t download_size = cache_strategy_->cache_element_size(count);

  std::vector<rosbaz::io::byte> download_buffer;
  download_buffer.resize(download_size);

  download(&(*download_buffer.begin()), offset, download_size);
  std::copy_n(download_buffer.begin(), count, buffer);

  std::lock_guard<std::mutex> lock_guard(cache_mutex_);
  cache_strategy_->update(std::move(download_buffer), offset);
}  // namespace io

}  // namespace io
}  // namespace rosbaz