#pragma once

#include <cstdint>
#include <set>

#include <boost/optional.hpp>
#include <ros/time.h>
#include <rosbag/query.h>
#include <rosbag/structures.h>

#include "rosbaz/bag.h"
#include "rosbaz/message_instance.h"

namespace rosbaz {

class View;

struct MessageRange {
  std::multiset<rosbag::IndexEntry>::const_iterator begin;
  std::multiset<rosbag::IndexEntry>::const_iterator end;
  const rosbag::ConnectionInfo *connection_info;
  const Bag *bag;
  rosbaz::io::IReader *reader;
};

struct ViewIterHelper {
  ViewIterHelper(std::multiset<rosbag::IndexEntry>::const_iterator _iter,
                 const MessageRange &_range);

  std::multiset<rosbag::IndexEntry>::const_iterator iter;
  const MessageRange *range;
};

struct ViewIterHelperCompare {
  bool operator()(ViewIterHelper const &a, ViewIterHelper const &b);
};

struct View {
public:
  struct ViewIterator
      : public std::iterator<std::forward_iterator_tag, MessageInstance> {
  public:
    using iterator = ViewIterator;
    using const_pointer = const MessageInstance *;

  private:
    friend class View;

    ViewIterator(const View &view, bool end = false) : m_view(&view) {
      if (!end) {
        populate();
      }
    }

    void populate() {
      m_message_instance.reset();
      for (const auto &range : m_view->m_ranges) {
        if (range.begin != range.end) {
          m_iters.emplace_back(range.begin, range);
        }
      }

      std::sort(m_iters.begin(), m_iters.end(), ViewIterHelperCompare());
    }

    void increment() {
      m_message_instance.reset();
      m_iters.back().iter++;

      if (m_iters.back().iter == m_iters.back().range->end) {
        m_iters.pop_back();
      }

      std::sort(m_iters.begin(), m_iters.end(), ViewIterHelperCompare());
    }

  public:
    iterator &operator++() {
      increment();
      return *this;
    }
    iterator operator++(int) {
      iterator tmp = *this;
      increment();
      return tmp;
    }

    bool operator==(const iterator &other) const {
      if (m_iters.empty()) {
        return other.m_iters.empty();
      }
      if (other.m_iters.empty()) {
        return false;
      }

      return m_iters.back().iter == other.m_iters.back().iter;
    }
    bool operator!=(const iterator &rhs) const { return !(*this == rhs); }

    value_type operator*() const {
      if (!m_message_instance) {
        auto it = m_iters.back();
        m_message_instance.emplace(MessageInstance{*it.range->connection_info,
                                                   *it.iter, *it.range->bag,
                                                   *it.range->reader});
      }
      return *m_message_instance;
    }
    const_pointer operator->() const {
      if (!m_message_instance) {
        auto it = m_iters.back();
        m_message_instance.emplace(MessageInstance{*it.range->connection_info,
                                                   *it.iter, *it.range->bag,
                                                   *it.range->reader});
      }
      return &(*m_message_instance);
    }

  protected:
    const View *m_view;
    std::vector<ViewIterHelper> m_iters;

    mutable boost::optional<MessageInstance> m_message_instance;
  };

  explicit View(Bag &bag, rosbaz::io::IReader &reader,
                ros::Time start_time = ros::TIME_MIN,
                ros::Time end_time = ros::TIME_MAX);

  explicit View(Bag &bag, rosbaz::io::IReader &reader,
                std::function<bool(rosbag::ConnectionInfo const *)> query,
                ros::Time const &start_time = ros::TIME_MIN,
                ros::Time const &end_time = ros::TIME_MAX);

  size_t size() const;

  ViewIterator begin() const;
  ViewIterator end() const;

  ros::Time getBeginTime();
  ros::Time getEndTime();

  std::vector<const rosbag::ConnectionInfo *> getConnections() const;

private:
  std::vector<MessageRange> m_ranges;
  const Bag *m_bag;
  rosbaz::io::IReader *m_reader;
};
} // namespace rosbaz