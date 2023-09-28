#include "rosbaz/view.h"

#include "rosbaz/bag.h"
#include "rosbaz/io/util.h"

#include <algorithm>
#include <assert.h>
#include <boost/type_index/type_index_facade.hpp>
#include <iterator>
#include <map>
#include <rosbag/query.h>
#include <rosbag/view.h>
#include <unordered_map>
#include <utility>

namespace rosbaz
{
View::iterator::iterator() = default;

View::iterator::~iterator() = default;

View::iterator::iterator(View& view, bool end) : view_(&view)
{
  if (!end)
  {
    populate();
  }
}

View::iterator::iterator(const iterator& i)
  : view_(i.view_), iters_(i.iters_), view_revision_(i.view_revision_), message_instance_{}
{
}

View::iterator& View::iterator::operator=(iterator const& i)
{
  if (this != &i)
  {
    view_ = i.view_;
    iters_ = i.iters_;
    view_revision_ = i.view_revision_;
    message_instance_.reset();
  }
  return *this;
}

void View::iterator::populate()
{
  assert(view_ != nullptr);

  iters_.clear();
  for (const auto& range : view_->ranges_)
  {
    if (range->begin != range->end)
    {
      iters_.emplace_back(range->begin, *range);
    }
  }

  std::sort(iters_.begin(), iters_.end(), ViewIterHelperCompare());
  view_revision_ = view_->view_revision_;
}

void View::iterator::populateSeek(std::multiset<rosbag::IndexEntry>::const_iterator iter)
{
  assert(view_ != nullptr);

  iters_.clear();
  for (const auto& range : view_->ranges_)
  {
    auto start = std::lower_bound(range->begin, range->end, iter->time, rosbag::IndexEntryCompare());
    if (start != range->end)
    {
      iters_.push_back(ViewIterHelper(start, *range));
    }
  }

  assert(!iters_.empty());
  std::sort(iters_.begin(), iters_.end(), ViewIterHelperCompare());
  while (iter != iters_.back().iter)
  {
    increment();
  }

  view_revision_ = view_->view_revision_;
}

bool View::iterator::equal(iterator const& other) const
{
  if (iters_.empty())
  {
    return other.iters_.empty();
  }
  if (other.iters_.empty())
  {
    return false;
  }

  return iters_.back().iter == other.iters_.back().iter;
}

void View::iterator::increment()
{
  assert(view_ != nullptr);

  message_instance_.reset();

  view_->update();

  assert(!iters_.empty()); // increment called on end iterator

  // Note, updating may have blown away our message-ranges and
  // replaced them in general the ViewIterHelpers are no longer
  // valid, but the iterator it stores should still be good.
  if (view_revision_ != view_->view_revision_)
  {
    populateSeek(iters_.back().iter);
  }

  iters_.back().iter++;

  if (iters_.back().iter == iters_.back().range->end)
  {
    iters_.pop_back();
  }

  std::sort(iters_.begin(), iters_.end(), ViewIterHelperCompare());
}

MessageInstance& View::iterator::dereference() const
{
  if (!message_instance_)
  {
    auto it = iters_.back();
    message_instance_.reset(view_->newMessageInstance(*it.range->connection_info, *it.iter, *it.range->bag_query->bag));
  }
  return *message_instance_;
}

View::View(const Bag& bag, const ros::Time& start_time, const ros::Time& end_time)
  : View(
        bag, [](rosbag::ConnectionInfo const*) { return true; }, start_time, end_time)
{
}

View::View(const Bag& bag, std::function<bool(rosbag::ConnectionInfo const*)> query, const ros::Time& start_time,
           const ros::Time& end_time)
{
  assert(bag.chunk_indices_parsed_);

  addQuery(bag, query, start_time, end_time);
}

View::~View() = default;

void View::updateQueries(BagQuery& q)
{
  const Bag& bag = *q.bag;
  for (const auto& connection_info : bag.connections_)
  {
    const rosbag::ConnectionInfo& connection = *connection_info.second;

    if (!q.query.getQuery()(&connection))
    {
      continue;
    }

    auto index_found = bag.connection_indexes_.find(connection.id);

    if (index_found == bag.connection_indexes_.end())
    {
      continue;
    }

    const auto& index_entries = index_found->second;

    rosbag::IndexEntry start_time_lookup_entry = { q.query.getStartTime(), 0, 0 };
    rosbag::IndexEntry end_time_lookup_entry = { q.query.getEndTime(), 0, 0 };

    auto begin = index_entries.lower_bound(start_time_lookup_entry);
    auto end = index_entries.upper_bound(end_time_lookup_entry);

    // Make sure we are at the right beginning
    while (begin != index_entries.begin() && begin->time >= q.query.getStartTime())
    {
      begin--;
      if (begin->time < q.query.getStartTime())
      {
        begin++;
        break;
      }
    }

    if (begin != end)
    {
      // todo: make faster with a map of maps
      bool found = false;
      for (auto& range : ranges_)
      {
        // If the topic and query are already in our ranges, we update
        if (range->bag_query == &q && range->connection_info->id == connection.id)
        {
          range->begin = begin;
          range->end = end;
          found = true;
          break;
        }
      }
      if (!found)
      {
        ranges_.emplace_back(std::make_unique<MessageRange>(begin, end, connection, q));
      }
    }
  }

  view_revision_++;
}

View::iterator View::begin()
{
  update();
  return View::iterator(*this);
}

View::iterator View::end()
{
  return View::iterator(*this, true);
}

size_t View::size()
{
  update();

  if (size_revision_ != view_revision_)
  {
    size_cache_ = 0;

    for (const auto& range : ranges_)
    {
      size_cache_ += rosbaz::io::narrow<uint32_t>(std::distance(range->begin, range->end));
    }

    size_revision_ = view_revision_;
  }

  return size_cache_;
}

ros::Time View::getBeginTime()
{
  update();

  ros::Time begin = ros::TIME_MAX;

  for (const auto& range : ranges_)
  {
    if (range->begin->time < begin)
    {
      begin = range->begin->time;
    }
  }

  return begin;
}

ros::Time View::getEndTime()
{
  update();

  ros::Time end = ros::TIME_MIN;

  for (const auto& range : ranges_)
  {
    auto e = range->end;
    e--;

    if (e->time > end)
    {
      end = e->time;
    }
  }

  return end;
}

void View::addQuery(const Bag& bag, const ros::Time& start_time, const ros::Time& end_time)
{
  addQuery(bag, rosbag::View::TrueQuery(), start_time, end_time);
}

void View::addQuery(const Bag& bag, boost::function<bool(rosbag::ConnectionInfo const*)> query,
                    const ros::Time& start_time, const ros::Time& end_time)
{
  assert(bag.chunk_indices_parsed_);

  queries_.emplace_back(std::make_unique<BagQuery>(bag, rosbag::Query(query, start_time, end_time), bag.bag_revision_));

  updateQueries(*queries_.back());
}

void View::update()
{
  for (auto& query : queries_)
  {
    const Bag& query_bag = *query->bag;

    if (query_bag.bag_revision_ != query->bag_revision)
    {
      updateQueries(*query);
      query->bag_revision = query_bag.bag_revision_;
    }
  }
}

std::vector<const rosbag::ConnectionInfo*> View::getConnections() const
{
  std::vector<const rosbag::ConnectionInfo*> connections;
  connections.reserve(ranges_.size());

  for (const auto& range : ranges_)
  {
    connections.push_back(range->connection_info);
  }

  return connections;
}

MessageInstance* View::newMessageInstance(const rosbag::ConnectionInfo& connection_info,
                                          rosbag::IndexEntry const& index, Bag const& bag) const
{
  return new MessageInstance(connection_info, index, bag);
}

}  // namespace rosbaz
