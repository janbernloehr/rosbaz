#pragma once

#include "rosbaz/internal/span.hpp"

#include <ostream>

namespace rosbag
{
struct ConnectionInfo;
}  // namespace rosbag

namespace rosbaz
{
class Bag;

namespace app
{
void write_bag_human_friendly(const rosbaz::Bag& bag, std::ostream& stream);
void write_bag_human_friendly(const rosbaz::Bag& bag,
                              const nonstd::span<const rosbag::ConnectionInfo* const> connections,
                              std::ostream& stream);

void write_bag_yaml(const rosbaz::Bag& bag, std::ostream& stream);
void write_bag_yaml(const rosbaz::Bag& bag, const nonstd::span<const rosbag::ConnectionInfo* const> connections,
                    std::ostream& stream);
}  // namespace app
}  // namespace rosbaz