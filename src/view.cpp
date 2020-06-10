#include "rosbaz/view.h"

#include <ros/console.h>

namespace rosbaz {

bool ViewIterHelperCompare::operator()(ViewIterHelper const &a,
                                       ViewIterHelper const &b) {
  return (a.iter)->time > (b.iter)->time;
}

ViewIterHelper::ViewIterHelper(
    std::multiset<rosbag::IndexEntry>::const_iterator _iter,
    const MessageRange &_range)
    : iter(_iter), range(&_range) {}

View::View(Bag &bag, rosbaz::io::IReader &reader, ros::Time start_time,
           ros::Time end_time)
    : View(
          bag, reader, [](rosbag::ConnectionInfo const *) { return true; },
          start_time, end_time) {}

View::View(Bag &bag, rosbaz::io::IReader &reader,
           std::function<bool(rosbag::ConnectionInfo const *)> query,
           const ros::Time &start_time, const ros::Time &end_time)
    : m_bag(&bag) {
  assert(bag.chunk_indices_parsed_);

  for (const auto &connection_info : bag.connections_) {
    if (!query(&connection_info.second)) {
      continue;
    }

    const auto &index_entries =
        bag.connection_indexes_.at(connection_info.first);

    rosbag::IndexEntry start_time_lookup_entry = {start_time, 0, 0};
    rosbag::IndexEntry end_time_lookup_entry = {end_time, 0, 0};

    auto begin = index_entries.lower_bound(start_time_lookup_entry);
    auto end = index_entries.upper_bound(end_time_lookup_entry);

    if (begin != end) {
      auto e = end;
      --e;

      m_ranges.emplace_back(
          MessageRange{begin, end, &connection_info.second, &bag, &reader});
    }
  }

  std::sort(m_ranges.begin(), m_ranges.end(),
            [](const MessageRange &a, const MessageRange &b) {
              return a.begin->time > b.begin->time;
            });

  for (const auto &range : m_ranges) {
    auto e = range.end;
    --e;

    ROS_DEBUG_STREAM(range.connection_info->topic
                     << ": " << std::distance(range.begin, range.end) << " :: "
                     << range.begin->time.sec << "." << range.begin->time.nsec
                     << " - " << e->time.sec << "." << e->time.nsec);
  }
}

View::ViewIterator View::begin() const { return View::ViewIterator(*this); }
View::ViewIterator View::end() const { return View::ViewIterator(*this, true); }

size_t View::size() const {
  size_t size = 0;

  for (const auto &range : m_ranges) {
    size += std::distance(range.begin, range.end);
  }

  return size;
}

ros::Time View::getBeginTime() {
  ros::Time begin = ros::TIME_MAX;

  for (const auto &range : m_ranges) {
    if (range.begin->time < begin) {
      begin = range.begin->time;
    }
  }

  return begin;
}

ros::Time View::getEndTime() {
  ros::Time end = ros::TIME_MIN;

  for (const auto &range : m_ranges) {
    auto e = range.end;
    e--;

    if (e->time > end) {
      end = e->time;
    }
  }

  return end;
}

std::vector<const rosbag::ConnectionInfo *> View::getConnections() const {
  return m_bag->getConnections();
}

} // namespace rosbaz