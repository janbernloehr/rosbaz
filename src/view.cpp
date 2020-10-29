#include "rosbaz/view.h"

#include <ros/console.h>
#include <rosbag/view.h>

#include "rosbaz/io/util.h"

namespace rosbaz
{
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

View::iterator::iterator(View& view, bool end) : view_(&view)
{
  if (!end)
  {
    populate();
  }
}

void View::iterator::populate()
{
  assert(view_ != nullptr);

  iters_.clear();
  for (const auto& range : view_->m_ranges)
  {
    if (range.begin != range.end)
    {
      iters_.emplace_back(range.begin, range);
    }
  }

  std::sort(iters_.begin(), iters_.end(), ViewIterHelperCompare());
  view_revision_ = view_->m_view_revision;
}

void View::iterator::populateSeek(std::multiset<rosbag::IndexEntry>::const_iterator iter)
{
  assert(view_ != nullptr);

  iters_.clear();
  for (const auto& range : view_->m_ranges)
  {
    auto start = std::lower_bound(range.begin, range.end, iter->time, rosbag::IndexEntryCompare());
    if (start != range.end)
    {
      iters_.push_back(ViewIterHelper(start, range));
    }
  }

  std::sort(iters_.begin(), iters_.end(), ViewIterHelperCompare());
  while (iter != iters_.back().iter)
  {
    increment();
  }

  view_revision_ = view_->m_view_revision;
}

void View::iterator::increment()
{
  assert(view_ != nullptr);

  message_instance_.reset();

  view_->update();

  // Note, updating may have blown away our message-ranges and
  // replaced them in general the ViewIterHelpers are no longer
  // valid, but the iterator it stores should still be good.
  if (view_revision_ != view_->m_view_revision)
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

MessageInstance& View::iterator::dereference() const
{
  if (!message_instance_)
  {
    auto it = iters_.back();
    message_instance_.emplace(MessageInstance{ *it.range->connection_info, *it.iter, it.range->bag_query->bag });
  }
  return *message_instance_;
}

bool ViewIterHelperCompare::operator()(ViewIterHelper const& a, ViewIterHelper const& b)
{
  return (a.iter)->time > (b.iter)->time;
}

ViewIterHelper::ViewIterHelper(std::multiset<rosbag::IndexEntry>::const_iterator _iter, const MessageRange& _range)
  : iter(_iter), range(&_range)
{
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

void View::updateQueries(BagQuery& q)
{
  const Bag& bag = q.bag;
  for (const auto& connection_info : bag.connections_)
  {
    const rosbag::ConnectionInfo& connection = connection_info.second;

    if (!q.query.getQuery()(&connection))
    {
      continue;
    }

    auto index_found = bag.connection_indexes_.find(connection_info.first);

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
      for (auto& range : m_ranges)
      {
        // If the topic and query are already in our ranges, we update
        if (range.bag_query == &q && range.connection_info->id == connection.id)
        {
          range.begin = begin;
          range.end = end;
          found = true;
          break;
        }
      }
      if (!found)
      {
        m_ranges.emplace_back(MessageRange{ begin, end, connection_info.second, q });
      }
    }

    m_view_revision++;
  }

  std::sort(m_ranges.begin(), m_ranges.end(),
            [](const MessageRange& a, const MessageRange& b) { return a.begin->time > b.begin->time; });

  for (const auto& range : m_ranges)
  {
    auto e = range.end;
    --e;

    ROS_DEBUG_STREAM(range.connection_info->topic << ": " << std::distance(range.begin, range.end)
                                                  << " :: " << range.begin->time.sec << "." << range.begin->time.nsec
                                                  << " - " << e->time.sec << "." << e->time.nsec);
  }
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

  if (m_size_revision != m_view_revision)
  {
    m_size_cache = 0;

    for (const auto& range : m_ranges)
    {
      m_size_cache += rosbaz::io::narrow<uint32_t>(std::distance(range.begin, range.end));
    }

    m_size_revision = m_view_revision;
  }

  return m_size_cache;
}

ros::Time View::getBeginTime()
{
  update();

  ros::Time begin = ros::TIME_MAX;

  for (const auto& range : m_ranges)
  {
    if (range.begin->time < begin)
    {
      begin = range.begin->time;
    }
  }

  return begin;
}

ros::Time View::getEndTime()
{
  update();

  ros::Time end = ros::TIME_MIN;

  for (const auto& range : m_ranges)
  {
    auto e = range.end;
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
  assert(bag.chunk_indices_parsed_);

  boost::function<bool(rosbag::ConnectionInfo const*)> query = rosbag::View::TrueQuery();

  m_queries.push_back(BagQuery(bag, rosbag::Query(query, start_time, end_time), bag.bag_revision_));

  updateQueries(m_queries.back());
}

void View::addQuery(const Bag& bag, boost::function<bool(rosbag::ConnectionInfo const*)> query,
                    const ros::Time& start_time, const ros::Time& end_time)
{
  assert(bag.chunk_indices_parsed_);

  m_queries.push_back(BagQuery(bag, rosbag::Query(query, start_time, end_time), bag.bag_revision_));

  updateQueries(m_queries.back());
}

void View::update()
{
  for (auto& query : m_queries)
  {
    const Bag& query_bag = query.bag;

    if (query_bag.bag_revision_ != query.bag_revision)
    {
      updateQueries(query);
      query.bag_revision = query_bag.bag_revision_;
    }
  }
}

std::vector<const rosbag::ConnectionInfo*> View::getConnections() const
{
  std::vector<const rosbag::ConnectionInfo*> connections;
  connections.reserve(m_ranges.size());

  for (const auto& range : m_ranges)
  {
    connections.push_back(range.connection_info);
  }

  return connections;
}

MessageRange::MessageRange(std::multiset<rosbag::IndexEntry>::const_iterator _begin,
                           std::multiset<rosbag::IndexEntry>::const_iterator _end,
                           const rosbag::ConnectionInfo& _connection_info, const BagQuery& _bag_query)
  : begin(_begin), end(_end), connection_info(&_connection_info), bag_query(&_bag_query)
{
}

}  // namespace rosbaz