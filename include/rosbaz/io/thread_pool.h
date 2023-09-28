#pragma once

#include <functional>
#include <iosfwd>
#include <mutex>
#include <thread>
#include <vector>

namespace rosbaz
{
namespace io
{
/// Spawn at most \p max_threads to process \p work_items.
void process_async(std::mutex& sync, std::vector<std::function<void(void)>>& work_items,
                   std::size_t max_threads = std::thread::hardware_concurrency());
}  // namespace io
}  // namespace rosbaz
