#include "rosbaz/io/thread_pool.h"

#include <future>
#include <ros/console.h>

namespace rosbaz
{
namespace io
{
void process_async(std::mutex& sync, std::vector<std::function<void(void)>>& work_items, std::size_t max_threads)
{
  assert(max_threads > 0);

  if (max_threads == 1)
  {
    for (auto&& f : work_items)
    {
      f();
    }
  }
  else
  {
    ROS_DEBUG_STREAM("Spawning " << max_threads << " threads to process " << work_items.size() << " work items");

    std::vector<std::future<void>> thread_pool;
    for (size_t i = 0; i < max_threads; ++i)
    {
      thread_pool.emplace_back(std::async(std::launch::async, [&sync, &work_items]() {
        while (true)
        {
          std::function<void(void)> f;
          {
            std::lock_guard<std::mutex> guard(sync);
            if (work_items.empty())
            {
              break;
            }
            f = std::move(work_items.back());
            work_items.pop_back();
          }

          f();
        }
      }));
    }

    for (auto&& f : thread_pool)
    {
      f.get();
    }
  }
}

}  // namespace io
}  // namespace rosbaz