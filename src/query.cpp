#include "rosbaz/query.h"

#include <ros/time.h>

namespace rosbaz
{
class Bag;

BagQuery::BagQuery(const Bag& _bag, const rosbag::Query& _query, uint32_t _bag_revision)
  : bag(&_bag), query(_query), bag_revision(_bag_revision)
{
}

MessageRange::MessageRange(std::multiset<rosbag::IndexEntry>::const_iterator _begin,
                           std::multiset<rosbag::IndexEntry>::const_iterator _end,
                           const rosbag::ConnectionInfo& _connection_info, const BagQuery& _bag_query)
  : begin(_begin), end(_end), connection_info(&_connection_info), bag_query(&_bag_query)
{
}

bool ViewIterHelperCompare::operator()(ViewIterHelper const& a, ViewIterHelper const& b)
{
  return (a.iter)->time > (b.iter)->time;
}

ViewIterHelper::ViewIterHelper(std::multiset<rosbag::IndexEntry>::const_iterator _iter, const MessageRange& _range)
  : iter(_iter), range(&_range)
{
}

}  // namespace rosbaz