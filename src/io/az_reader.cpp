#include "rosbaz/io/az_reader.h"

#include <ros/console.h>

#include "rosbaz/exceptions.h"
#include "rosbaz/io/io_helpers.h"

namespace {
std::shared_ptr<azure::storage_lite::storage_credential>
create_credential(const rosbaz::AzBlobUrl &blob_url,
                  const std::string &account_key = "",
                  const std::string &token = "") {
  std::shared_ptr<azure::storage_lite::storage_credential> credential;

  if (!token.empty()) {
    credential = std::make_shared<azure::storage_lite::token_credential>(token);
  } else if (!blob_url.sas_token.empty()) {
    credential = std::make_shared<
        azure::storage_lite::shared_access_signature_credential>(
        blob_url.sas_token);
  } else if (!account_key.empty()) {
    credential = std::make_shared<azure::storage_lite::shared_key_credential>(
        blob_url.account_name, account_key);
  } else {
    throw rosbaz::MissingCredentialsException(
        "You must provide either a bearer token, a sas "
        "token, or an account key.");
  }

  return credential;
}
} // namespace

namespace rosbaz {

namespace io {

AzReader::AzReader(const AzBlobUrl &blob_url, const std::string &account_key,
                   const std::string &token)
    : m_container(blob_url.container_name), m_blob(blob_url.blob_name) {
  bool use_https = blob_url.schema == "https";
  int connection_count = 2;

  auto credential = create_credential(blob_url, account_key, token);

  // Setup the client
  auto account = std::make_shared<azure::storage_lite::storage_account>(
      blob_url.account_name, credential, use_https, blob_url.blob_endpoint);
  m_client = std::make_shared<azure::storage_lite::blob_client>(
      account, connection_count);
}

size_t AzReader::size() {
  auto ret = m_client->get_blob_properties(m_container, m_blob).get();

  if (!ret.success()) {
    std::stringstream msg;
    msg << "Failed to fetch container properties, Error: " << ret.error().code
        << ", " << ret.error().code_name << std::endl;
    throw rosbaz::IoException(msg.str());
  }

  m_num_requests += 1;

  return ret.response().size;
}

void AzReader::read_fixed(rosbaz::io::byte *buffer, size_t offset,
                          size_t count) {
  auto cache_found = std::find_if(
      m_cache.begin(), m_cache.end(), [offset, count](const CacheEntry &entry) {
        return ((entry.offset <= offset) &&
                (offset + count <= entry.offset + entry.data.size()));
      });

  if (cache_found == m_cache.end()) {
    const size_t cache_size = std::max<size_t>(1024, count);
    CacheEntry entry;
    entry.offset = offset;
    entry.data.resize(cache_size);

    auto ret = m_client
                   ->download_blob_to_buffer(
                       m_container, m_blob, offset, cache_size,
                       reinterpret_cast<char *>(&(*entry.data.begin())), 2)
                   .get();

    if (!ret.success()) {
      std::stringstream msg;
      msg << "Failed to download blob, Error: " << ret.error().code << ", "
          << ret.error().code_name << std::endl;
      throw rosbaz::IoException(msg.str());
    }

    m_num_requests += 1;
    m_num_bytes += cache_size;
    ROS_DEBUG_STREAM("Read " << offset << ":" << cache_size);

    m_cache.emplace_back(std::move(entry));
    cache_found = m_cache.end() - 1;
  }

  std::copy_n(cache_found->data.begin() + (offset - cache_found->offset), count,
              buffer);
}

} // namespace io
} // namespace rosbaz