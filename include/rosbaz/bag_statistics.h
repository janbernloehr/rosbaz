#pragma once

#include "rosbaz/internal/nonstd/span.hpp"

#include <boost/optional.hpp>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace rosbag
{
struct ConnectionInfo;
}  // namespace rosbag

namespace rosbaz
{
class Bag;

struct BagMessageTypeInfo
{
  std::string datatype;
  std::string md5sum;
};

struct BagMessageTopicInfo
{
  std::string topic;
  std::uint32_t num_messages;
  boost::optional<double> frequency;
  std::string datatype;
};

class BagStatistics
{
public:
  explicit BagStatistics(const rosbaz::Bag& bag);

  std::uint32_t getTotalMessageCount();

  std::vector<BagMessageTypeInfo> getMessageTypeInfos();
  std::vector<BagMessageTypeInfo>
  getMessageTypeInfos(const nonstd::span<const rosbag::ConnectionInfo* const> connections);

  std::vector<BagMessageTopicInfo> getMessageTopicInfos();
  std::vector<BagMessageTopicInfo>
  getMessageTopicInfos(const nonstd::span<const rosbag::ConnectionInfo* const> connections);

private:
  const std::map<uint32_t, uint32_t>& getCacheConnectionIdToCount();

  std::uint32_t getMessageCountForConnectionId(std::uint32_t id);

  boost::optional<double> getMessageFrequencyForConnectionId(std::uint32_t id);

  const rosbaz::Bag& bag_;

  std::map<uint32_t, uint32_t> cache_connection_id_to_count_{};
  bool cache_connection_id_to_count_valid_{ false };
};
}  // namespace rosbaz