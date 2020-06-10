#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "blob/blob_client.h"

#include "rosbaz/blob_url.h"
#include "rosbaz/io/reader.h"

namespace rosbaz {

namespace io {

struct CacheEntry {
  std::vector<byte> data;
  std::uint64_t offset;
};

class AzReader : public IReader {
public:
  explicit AzReader(const AzBlobUrl &blob_url,
                    const std::string &account_key = "",
                    const std::string &token = "");

  size_t size() override;

  //   static std::unique_ptr<IReader> open(const std::string& file_path);

  std::int32_t num_requests() const { return m_num_requests; }
  std::int64_t num_bytes() const { return m_num_bytes; }

private:
  void read_fixed(rosbaz::io::byte *buffer, size_t offset,
                  size_t count) override;

  std::string m_container{};
  std::string m_blob{};
  std::shared_ptr<azure::storage_lite::blob_client> m_client{};

  std::int32_t m_num_requests{0};
  std::int64_t m_num_bytes{0};

  // TODO: Make cache size configurable
  // TODO: Use circular buffer to bound memory usage
  std::vector<CacheEntry> m_cache;
};

} // namespace io
} // namespace rosbaz