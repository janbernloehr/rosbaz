#pragma once

#include <cstdint>
#include <set>

#include <boost/optional.hpp>
#include <ros/time.h>
#include <rosbag/query.h>
#include <rosbag/structures.h>

#include "rosbaz/bag.h"
#include "rosbaz/message_instance.h"

namespace rosbaz
{
class View;

struct MessageRange
{
  std::multiset<rosbag::IndexEntry>::const_iterator begin;
  std::multiset<rosbag::IndexEntry>::const_iterator end;
  const rosbag::ConnectionInfo* connection_info;
  const Bag* bag;
  rosbaz::io::IReader* reader;
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

struct View
{
public:
  struct iterator : public std::iterator<std::forward_iterator_tag, MessageInstance>
  {
  public:
    using const_pointer = const MessageInstance*;

  private:
    friend class View;

    iterator(const View& view, bool end = false);

    void populate();

    void increment();

  public:
    iterator() = default;
    iterator(iterator const& i);
    iterator& operator=(iterator const& i);

    iterator& operator++();
    iterator operator++(int);

    bool operator==(const iterator& other) const;
    bool operator!=(const iterator& rhs) const;

    value_type operator*() const;
    const_pointer operator->() const;

  protected:
    const View* view_{ nullptr };
    std::vector<ViewIterHelper> iters_{};

    mutable boost::optional<MessageInstance> message_instance_{};
  };

  explicit View(const Bag& bag, ros::Time start_time = ros::TIME_MIN, ros::Time end_time = ros::TIME_MAX);

  explicit View(const Bag& bag, std::function<bool(rosbag::ConnectionInfo const*)> query,
                ros::Time const& start_time = ros::TIME_MIN, ros::Time const& end_time = ros::TIME_MAX);

  size_t size() const;

  iterator begin() const;
  iterator end() const;

  ros::Time getBeginTime();
  ros::Time getEndTime();

  std::vector<const rosbag::ConnectionInfo*> getConnections() const;

private:
  std::vector<MessageRange> m_ranges;
  const Bag* m_bag;
  rosbaz::io::IReader* m_reader;
};
}  // namespace rosbaz