#pragma once

#include <cstdint>
#include <rosbag/query.h>

#include "rosbaz/bag.h"

namespace rosbaz
{
struct BagQuery
{
  BagQuery(const Bag& _bag, const rosbag::Query& _query, uint32_t _bag_revision);

  std::reference_wrapper<const Bag> bag;
  rosbag::Query query;
  uint32_t bag_revision;
};

}  // namespace rosbaz