#pragma once

#include <functional>
#include <mutex>
#include <thread>
#include <vector>

namespace rosbaz
{
namespace io
{
/// Spawn at most \p max_threads to call the given functions.
void process_async(std::mutex& sync, std::vector<std::function<void(void)>>& work,
                   std::size_t max_threads = std::thread::hardware_concurrency());
}  // namespace io
}  // namespace rosbaz