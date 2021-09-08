#pragma once

#include "rosbaz/bag.h"

#include <ostream>

namespace rosbaz
{
namespace app
{
void write_bag_human_friendly(const rosbaz::Bag& bag, std::ostream& stream);

void write_bag_yaml(const rosbaz::Bag& bag, std::ostream& stream);
}  // namespace app
}  // namespace rosbaz