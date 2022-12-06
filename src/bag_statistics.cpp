#include "rosbaz/bag_statistics.h"

#include "rosbaz/bag.h"

#include <algorithm>
#include <boost/accumulators/framework/accumulator_set.hpp>
#include <boost/accumulators/framework/extractor.hpp>
#include <boost/accumulators/framework/features.hpp>
#include <boost/accumulators/statistics/median.hpp>
#include <boost/fusion/iterator/deref.hpp>
#include <numeric>
#include <ros/duration.h>
#include <ros/time.h>
#include <rosbag/structures.h>
#include <set>
#include <unordered_map>

namespace rosbaz
{
BagStatistics::BagStatistics(const rosbaz::Bag& bag) : bag_{ bag }
{
}

std::uint32_t BagStatistics::getMessageCountForConnectionId(uint32_t id)
{
  std::uint32_t count = 0;

  for (const auto& chunk_info : bag_.chunks_)
  {
    if (chunk_info.connection_counts.count(id) != 0)
    {
      count += chunk_info.connection_counts.at(id);
    }
  }

  return count;
}

boost::optional<double> BagStatistics::getMessageFrequencyForConnectionId(uint32_t id)
{
  if (bag_.connection_indexes_.count(id) == 0)
  {
    return {};
  }

  const auto& indexes = bag_.connection_indexes_.at(id);
  if (indexes.size() < 2)
  {
    return {};
  }

  boost::optional<ros::Time> last_time;
  boost::accumulators::accumulator_set<double, boost::accumulators::features<boost::accumulators::tag::median>> acc;

  for (const auto& entry : indexes)
  {
    if (last_time)
    {
      acc((entry.time - *last_time).toSec());
    }

    last_time = entry.time;
  }

  const double median_period = boost::accumulators::median(acc);

  if (median_period > 0.)
  {
    return 1.0 / median_period;
  }

  return {};
}

std::vector<BagMessageTypeInfo> BagStatistics::getMessageTypeInfos()
{
  const auto connections = bag_.getConnections();
  return getMessageTypeInfos(connections);
}

std::vector<BagMessageTypeInfo>
BagStatistics::getMessageTypeInfos(const nonstd::span<const rosbag::ConnectionInfo* const> connections)
{
  std::vector<BagMessageTypeInfo> result;

  for (const auto* connection_info : connections)
  {
    if (std::find_if(result.begin(), result.end(), [&connection_info](const auto& other) {
          return other.datatype == connection_info->datatype;
        }) != result.end())
    {
      continue;
    }
    result.emplace_back(BagMessageTypeInfo{ connection_info->datatype, connection_info->md5sum });
  }

  std::sort(result.begin(), result.end(), [](const auto& a, const auto& b) { return a.datatype < b.datatype; });

  return result;
}

std::vector<BagMessageTopicInfo> BagStatistics::getMessageTopicInfos()
{
  const auto connections = bag_.getConnections();
  return getMessageTopicInfos(connections);
}

std::vector<BagMessageTopicInfo>
BagStatistics::getMessageTopicInfos(const nonstd::span<const rosbag::ConnectionInfo* const> connections)
{
  const auto& cache = getCacheConnectionIdToCount();
  std::vector<BagMessageTopicInfo> result;

  for (const auto* connection_info : connections)
  {
    result.emplace_back(BagMessageTopicInfo{ connection_info->topic, cache.at(connection_info->id),
                                             getMessageFrequencyForConnectionId(connection_info->id),
                                             connection_info->datatype });
  }

  std::sort(result.begin(), result.end(), [](const auto& a, const auto& b) { return a.topic < b.topic; });

  return result;
}

const std::map<uint32_t, uint32_t>& BagStatistics::getCacheConnectionIdToCount()
{
  if (!cache_connection_id_to_count_valid_)
  {
    for (const auto* connection_info : bag_.getConnections())
    {
      const uint32_t count = getMessageCountForConnectionId(connection_info->id);
      cache_connection_id_to_count_[connection_info->id] = count;
    }

    cache_connection_id_to_count_valid_ = true;
  }

  return cache_connection_id_to_count_;
}

std::uint32_t BagStatistics::getTotalMessageCount()
{
  const auto& cache = getCacheConnectionIdToCount();

  return std::accumulate(cache.begin(), cache.end(), uint32_t{ 0 },
                         [](uint32_t a, const auto& b) { return a + b.second; });
}

}  // namespace rosbaz