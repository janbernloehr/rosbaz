#pragma once

#include <cstdint>
#include <memory>
#include <set>
#include <vector>

#include <boost/iterator/iterator_facade.hpp>
#include <ros/time.h>
#include <rosbag/query.h>
#include <rosbag/structures.h>

#include "rosbaz/bag.h"
#include "rosbaz/query.h"
#include "rosbaz/message_instance.h"

namespace rosbaz
{
struct View;

struct View
{
public:
  /// An iterator that points to a MessageInstance from a bag
  ///
  /// This iterator stores the MessageInstance that it is returning a
  /// reference to.  If you increment the iterator, that
  /// MessageInstance is destroyed.  You should never store the
  /// pointer to this reference.
  class iterator : public boost::iterator_facade<iterator, MessageInstance, boost::forward_traversal_tag>
  {
  public:
    iterator();
    iterator(iterator const& i);
    iterator& operator=(iterator const& i);
    ~iterator();

  private:
    friend struct View;
    friend class boost::iterator_core_access;

    iterator(View& view, bool end = false);

    void populate();
    void populateSeek(std::multiset<rosbag::IndexEntry>::const_iterator iter);

    void increment();

    bool equal(iterator const& other) const;

    MessageInstance& dereference() const;

  protected:
    View* view_{ nullptr };
    std::vector<ViewIterHelper> iters_{};
    uint32_t view_revision_;

    mutable std::unique_ptr<MessageInstance> message_instance_{};
  };

  using const_iterator = iterator;

  /// Create a view on a bag
  ///
  /// \param[in] bag             The bag file on which to run this query
  /// \param[in] start_time      The beginning of the time range for the query
  /// \param[in] end_time        The end of the time range for the query
  explicit View(const Bag& bag, const ros::Time& start_time = ros::TIME_MIN, const ros::Time& end_time = ros::TIME_MAX);

  /// Create a view on a bag
  ///
  /// \param[in] bag             The bag file on which to run this query
  /// \param[in] query           The actual query to evaluate which connections to include
  /// \param[in] start_time      The beginning of the time range for the query
  /// \param[in] end_time        The end of the time range for the query
  explicit View(const Bag& bag, std::function<bool(rosbag::ConnectionInfo const*)> query,
                const ros::Time& start_time = ros::TIME_MIN, const ros::Time& end_time = ros::TIME_MAX);

  View(const View& view) = delete;
  View& operator=(const View& view) = delete;

  ~View();

  size_t size();

  iterator begin();
  iterator end();

  ros::Time getBeginTime();
  ros::Time getEndTime();

  /// Add a query to a view
  ///
  /// \param[in] bag        The bag file on which to run this query
  /// \param[in] start_time The beginning of the time range for the query
  /// \param[in] end_time   The end of the time range for the query
  void addQuery(const Bag& bag, const ros::Time& start_time = ros::TIME_MIN, const ros::Time& end_time = ros::TIME_MAX);

  /// Add a query to a view
  ///
  /// \param[in] bag        The bag file on which to run this query
  /// \param[in] query      The actual query to evaluate which connections to include
  /// \param[in] start_time The beginning of the time range for the query
  /// \param[in] end_time   The end of the time range for the query
  void addQuery(Bag const& bag, boost::function<bool(rosbag::ConnectionInfo const*)> query,
                const ros::Time& start_time = ros::TIME_MIN, const ros::Time& end_time = ros::TIME_MAX);

  std::vector<const rosbag::ConnectionInfo*> getConnections() const;

private:
  void updateQueries(BagQuery& q);
  void update();

  std::vector<std::unique_ptr<MessageRange>> ranges_{};
  std::vector<std::unique_ptr<BagQuery>> queries_{};
  uint32_t view_revision_{ 0 };

  uint32_t size_cache_{ 0 };
  uint32_t size_revision_{ 0 };

  MessageInstance* newMessageInstance(const rosbag::ConnectionInfo& connection_info, rosbag::IndexEntry const& index,
                                      Bag const& bag) const;
};
}  // namespace rosbaz