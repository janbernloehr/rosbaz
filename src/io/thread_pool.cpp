#include "rosbaz/io/thread_pool.h"

#include <future>

namespace rosbaz
{
namespace io
{
void process_async(std::mutex& sync, std::vector<std::function<void(void)>>& work,
                   std::size_t max_threads)
{
  std::vector<std::future<void>> thread_pool;
  for (size_t i = 0; i < max_threads; ++i)
  {
    thread_pool.emplace_back(std::async(std::launch::async, [&sync, &work]() {
      while (true)
      {
        std::function<void(void)> f;
        {
          std::lock_guard<std::mutex> guard(sync);
          if (work.empty())
          {
            break;
          }
          f = std::move(work.back());
          work.pop_back();
        }

        f();
      }
    }));
  }

  // the future dtor will wait for the completion here.
}

}  // namespace io
}  // namespace rosbaz