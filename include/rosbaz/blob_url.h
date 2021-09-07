#pragma once

#include <string>

namespace rosbaz
{
struct AzBlobUrl
{
  std::string schema{};
  std::string account_name{};
  std::string blob_endpoint{};
  std::string container_name{};
  std::string blob_name{};
  std::string sas_token{};

  static AzBlobUrl parse(const std::string& url);
};

bool is_url(const std::string& path);

}  // namespace rosbaz