#include "rosbaz/io/az_reader.h"

#include <ros/console.h>

#include <boost/make_unique.hpp>

#include "rosbaz/exceptions.h"
#include "rosbaz/io/io_helpers.h"
#include "rosbaz/io/small_element_cache.h"

namespace
{
std::shared_ptr<azure::storage_lite::storage_credential> create_credential(const rosbaz::AzBlobUrl& blob_url,
                                                                           const std::string& account_key = "",
                                                                           const std::string& token = "")
{
  std::shared_ptr<azure::storage_lite::storage_credential> credential;

  if (!token.empty())
  {
    credential = std::make_shared<azure::storage_lite::token_credential>(token);
  }
  else if (!blob_url.sas_token.empty())
  {
    credential = std::make_shared<azure::storage_lite::shared_access_signature_credential>(blob_url.sas_token);
  }
  else if (!account_key.empty())
  {
    credential = std::make_shared<azure::storage_lite::shared_key_credential>(blob_url.account_name, account_key);
  }
  else
  {
    throw rosbaz::MissingCredentialsException("You must provide either a bearer token, a sas "
                                              "token, or an account key.");
  }

  return credential;
}
}  // namespace

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
  const bool use_https = blob_url.schema == "https";
  const int connection_count = static_cast<int>(std::thread::hardware_concurrency());

  auto credential = create_credential(blob_url, account_key, token);

  // Setup the client
  auto account = std::make_shared<azure::storage_lite::storage_account>(blob_url.account_name, credential, use_https,
                                                                        blob_url.blob_endpoint);
  client_ = std::make_shared<azure::storage_lite::blob_client>(account, connection_count);
}

std::string AzReader::filepath()
{
  return client_->account()->get_url(azure::storage_lite::storage_account::service::blob).to_string() + "/" +
         container_ + "/" + blob_;
}

size_t AzReader::size()
{
  auto ret = client_->get_blob_properties(container_, blob_).get();

  if (!ret.success())
  {
    std::stringstream msg;
    msg << "Failed to fetch container properties, Error: " << ret.error().code << ", " << ret.error().code_name
        << std::endl;
    throw rosbaz::IoException(msg.str());
  }

  num_requests_ += 1;

  return ret.response().size;
}

void AzReader::download(rosbaz::io::byte* buffer, size_t offset, size_t count)
{
  auto ret =
      client_->download_blob_to_buffer(container_, blob_, offset, count, reinterpret_cast<char*>(buffer), 2).get();

  if (!ret.success())
  {
    std::stringstream msg;
    msg << "Failed to download blob, Error: " << ret.error().code << ", " << ret.error().code_name << std::endl;
    throw rosbaz::IoException(msg.str());
  }

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