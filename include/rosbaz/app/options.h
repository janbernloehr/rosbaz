#pragma once

#include <string>
#include <boost/optional.hpp>

namespace rosbaz
{
namespace app
{
struct CommonOptions
{
  std::string account_key{};
  std::string token{};

  bool print_transfer_stats{ false };
};

struct InfoOptions
{
  std::string file_or_blob_url{};

  bool topic_message_frequency_statistics = false;
  bool yaml_output = false;
};

struct PlayOptions
{
  std::string file_or_blob_url{};

  boost::optional<float> start_offset{};
  boost::optional<float> duration{};

  bool loop = false;
  bool start_paused = false;
  bool is_quiet = false;
};
}  // namespace app
}  // namespace rosbaz
