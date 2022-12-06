#include "rosbaz/bag.h"
#include "rosbaz/io/io_helpers.h"
#include "rosbaz/io/stream_reader.h"
#include "rosbaz/view.h"

#include <benchmark/benchmark.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MultiEchoLaserScan.h>

template <class ViewT, class MessageT, class BagT>
void runBenchmark(benchmark::State& state, const BagT& bag, const std::string& topic_name)
{
  ViewT view{ bag, rosbag::TopicQuery(topic_name) };

  while (state.KeepRunningBatch(view.size()))
  {
    for (const auto m : view)
    {
      typename MessageT::ConstPtr i = m.template instantiate<MessageT>();
      benchmark::DoNotOptimize(i);
      benchmark::ClobberMemory();
    }
  }
}

static void BM_rosbag_chatty(benchmark::State& state)
{
  rosbag::Bag bag{ "b0-2014-07-11-10-58-16-decompressed.bag" };
  runBenchmark<rosbag::View, sensor_msgs::Imu>(state, bag, "imu");
}
BENCHMARK(BM_rosbag_chatty);

static void BM_rosbaz_chatty(benchmark::State& state)
{
  rosbaz::Bag bag{ rosbaz::Bag::read(rosbaz::io::StreamReader::open("b0-2014-07-11-10-58-16-decompressed.bag")) };
  runBenchmark<rosbaz::View, sensor_msgs::Imu>(state, bag, "imu");
}
BENCHMARK(BM_rosbaz_chatty);

static void BM_rosbag_chunky(benchmark::State& state)
{
  rosbag::Bag bag{ "b0-2014-07-11-10-58-16-decompressed.bag" };
  runBenchmark<rosbag::View, sensor_msgs::MultiEchoLaserScan>(state, bag,
                                                              "horizontal_"
                                                              "laser_2d");
}
BENCHMARK(BM_rosbag_chunky);

static void BM_rosbaz_chunky(benchmark::State& state)
{
  rosbaz::Bag bag{ rosbaz::Bag::read(rosbaz::io::StreamReader::open("b0-2014-07-11-10-58-16-decompressed.bag")) };
  runBenchmark<rosbaz::View, sensor_msgs::MultiEchoLaserScan>(state, bag,
                                                              "horizontal_"
                                                              "laser_2d");
}
BENCHMARK(BM_rosbaz_chunky);

static void BM_rosbag_open(benchmark::State& state)
{
  for (auto _ : state)
  {
    rosbag::Bag bag{ "b0-2014-07-11-10-58-16-decompressed.bag" };
    benchmark::DoNotOptimize(bag);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(BM_rosbag_open);

static void BM_rosbaz_open(benchmark::State& state)
{
  for (auto _ : state)
  {
    rosbaz::Bag bag{ rosbaz::Bag::read(rosbaz::io::StreamReader::open("b0-2014-07-11-10-58-16-decompressed.bag")) };
    benchmark::DoNotOptimize(bag);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(BM_rosbaz_open);

BENCHMARK_MAIN();