#include "rosbaz/query.h"

namespace rosbaz
{
BagQuery::BagQuery(const Bag& _bag, const rosbag::Query& _query, uint32_t _bag_revision)
  : bag(_bag), query(_query), bag_revision(_bag_revision)
{
}
}  // namespace rosbaz