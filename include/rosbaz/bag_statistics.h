#pragma once

#include <cstdint>
#include <boost/optional.hpp>

#include "rosbaz/bag.h"

namespace rosbaz
{
struct BagMessageTypeInfo
{
  std::string datatype;
  std::string md5sum;

  bool operator==(const BagMessageTypeInfo& other) const;
  friend bool operator<(const BagMessageTypeInfo& l, const BagMessageTypeInfo& r)
  {
    return (l.datatype < r.datatype) || ((l.datatype == r.datatype) && (l.md5sum < r.md5sum));
  }
};

struct BagMessageTopicInfo
{
  std::string topic;
  std::uint32_t num_messages;
  boost::optional<double> frequency;
  std::string datatype;
};

class BagSatistics
{
public:
  explicit BagSatistics(const rosbaz::Bag& bag);

  std::uint32_t getTotalMessageCount();

  std::set<BagMessageTypeInfo> getMessageTypeInfos();

  std::vector<BagMessageTopicInfo> getMessageTopicInfos();

private:
  const std::map<uint32_t, uint32_t>& getCacheConnectionIdToCount();

  std::uint32_t getMessageCountForConnectionId(std::uint32_t id);

  boost::optional<double> getMessageFrequencyForConnectionId(std::uint32_t id);

  const rosbaz::Bag& bag_;

  std::map<uint32_t, uint32_t> cache_connection_id_to_count_{};
  bool cache_connection_id_to_count_valid_{ false };
};
}  // namespace rosbaz