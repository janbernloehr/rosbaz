#pragma once

#include <cstdint>
#include <rosbag/query.h>
#include <rosbag/structures.h>
#include <set>

namespace rosbaz
{
class Bag;

struct BagQuery
{
  BagQuery(const Bag& _bag, const rosbag::Query& _query, uint32_t _bag_revision);

  const Bag* bag;
  rosbag::Query query;
  uint32_t bag_revision;
};

struct MessageRange
{
  MessageRange(std::multiset<rosbag::IndexEntry>::const_iterator _begin,
               std::multiset<rosbag::IndexEntry>::const_iterator _end, const rosbag::ConnectionInfo& _connection_info,
               const BagQuery& _bag_query);

  std::multiset<rosbag::IndexEntry>::const_iterator begin;
  std::multiset<rosbag::IndexEntry>::const_iterator end;
  const rosbag::ConnectionInfo* connection_info;
  const BagQuery* bag_query{ nullptr };
};

struct ViewIterHelper
{
  ViewIterHelper(std::multiset<rosbag::IndexEntry>::const_iterator _iter, const MessageRange& _range);

  std::multiset<rosbag::IndexEntry>::const_iterator iter;
  const MessageRange* range;
};

struct ViewIterHelperCompare
{
  bool operator()(ViewIterHelper const& a, ViewIterHelper const& b);
};

}  // namespace rosbaz