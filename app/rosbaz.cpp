#include <regex>

#include <boost/optional.hpp>
#include <boost/optional/optional_io.hpp>

#include "cli11/CLI11.hpp"

#include <ros/console.h>
#include <ros/ros.h>
#include <rosbag/time_translator.h>

#include "terminal/pausable_context.h"
#include "terminal/progress_bar.h"

#include "rosbaz/bag_parsing/bag.h"
#include "rosbaz/bag_parsing/view.h"
#include "rosbaz/blob_url.h"
#include "rosbaz/io/az_reader.h"

namespace container {

template <class T>
inline std::istream &operator>>(std::istream &stream,
                                boost::optional<T> &optional) {
  optional.emplace();
  stream >> optional.value();
  return stream;
}

} // namespace container

struct CommonOptions {
  std::string account_key{};
  std::string token{};
};

struct InfoOptions {
  std::string blob_url{};
};

struct PlayOptions {
  std::string blob_url{};

  boost::optional<float> start_offset{};
  boost::optional<float> duration{};

  bool loop = false;
  bool start_paused = false;
  bool is_quiet = false;
};

namespace {

void print_bag(const rosbaz::bag_parsing::AzBag &bag) {
  auto start_time = bag.getBeginTime();
  auto end_time = bag.getEndTime();
  auto duration = end_time - start_time;

  std::cout << "duration: " << duration.toSec() << "s\n";
  std::cout << "start:    " << start_time.sec << "." << start_time.nsec << "\n";
  std::cout << "end:      " << end_time.sec << "." << end_time.nsec << "\n";
  std::cout << "size:     "
            << static_cast<float>(bag.getFileSize()) / 1024.f / 1024.f / 1024.f
            << " GB\n";

  std::map<uint32_t, uint32_t> total_connection_counts;

  uint32_t message_count = 0;

  for (const auto *connection_info : bag.getConnections()) {
    const uint32_t count =
        bag.getMessageCountForConnectionId(connection_info->id);
    total_connection_counts[connection_info->id] = count;
    message_count += count;
  }

  std::cout << "messages: " << message_count << "\n";
  std::cout << "chunks:   " << bag.getChunkCount() << "\n";

  std::cout << "types:";
  size_t max_type_name_length = 0;
  std::string prefix = "    ";

  std::map<std::string, std::string> data_type_2_md5;
  for (const auto *connection_info : bag.getConnections()) {
    max_type_name_length =
        std::max(max_type_name_length, connection_info->datatype.size());
    data_type_2_md5[connection_info->datatype] = connection_info->md5sum;
  }

  for (const auto &data_type_md5 : data_type_2_md5) {
    std::string padded_name = data_type_md5.first;
    padded_name.append(max_type_name_length - padded_name.size(), ' ');
    std::cout << prefix << padded_name << " [" << data_type_md5.second << "]\n";
    prefix = "          ";
  }

  std::cout << "topics:";
  size_t max_topic_name_length = 0;

  std::map<std::string, std::tuple<std::uint32_t, std::string>>
      topic_2_count_and_type;
  for (const auto *connection_info : bag.getConnections()) {
    max_topic_name_length =
        std::max(max_topic_name_length, connection_info->topic.size());
    topic_2_count_and_type[connection_info->topic] =
        std::make_tuple(total_connection_counts[connection_info->id],
                        connection_info->datatype);
  }

  prefix = "   ";

  for (const auto &topic_count_type : topic_2_count_and_type) {
    std::string padded_name = topic_count_type.first;
    padded_name.append(max_topic_name_length - padded_name.size(), ' ');
    std::cout << prefix << padded_name << " "
              << std::get<0>(topic_count_type.second)
              << " msgs   : " << std::get<1>(topic_count_type.second) << "\n";

    prefix = "          ";
  }
}

void info_command(const CommonOptions &common_options,
                  const InfoOptions &info_options) {
  auto url = rosbaz::AzBlobUrl::parse(info_options.blob_url);

  rosbaz::io::AzReader az_reader{url, common_options.account_key,
                                 common_options.token};
  auto az_bag = rosbaz::bag_parsing::AzBag::read(az_reader, false);
  print_bag(az_bag);

  ROS_INFO_STREAM("Requests: " << az_reader.num_requests() << " bytes "
                               << az_reader.num_bytes());
}

void play_command(const CommonOptions &common_options,
                  const PlayOptions &play_options) {
  // setup node handle
  std::map<std::string, std::string> mapping{};
  ros::init(mapping, "rosbaz");
  ros::NodeHandle node_handle{};

  auto url = rosbaz::AzBlobUrl::parse(play_options.blob_url);
  rosbaz::io::AzReader az_reader{url, common_options.account_key,
                                 common_options.token};
  auto az_bag = rosbaz::bag_parsing::AzBag::read(az_reader);

  // we first create a full blown view to obtain the time range of the messages
  // and then filter that view
  rosbaz::bag_parsing::View full_view{az_bag, az_reader};

  const auto initial_time =
      full_view.getBeginTime() +
      ros::Duration(play_options.start_offset.value_or(0));
  ros::Time finish_time = ros::TIME_MAX;
  if (play_options.duration) {
    finish_time = initial_time + ros::Duration(*play_options.duration);
  }

  rosbaz::bag_parsing::View filtered_view{az_bag, az_reader, initial_time,
                                          finish_time};

  std::unordered_map<std::string, ros::Publisher> publishers;
  for (const auto *connection_info : filtered_view.getConnections()) {
    ros::AdvertiseOptions options(
        connection_info->topic, 50, connection_info->md5sum,
        connection_info->datatype, connection_info->msg_def);
    publishers[connection_info->topic] = node_handle.advertise(options);
  }

  terminal::PausableContext pausable_context{play_options.start_paused};

  auto evaluate_tick = [&pausable_context]() {
    return pausable_context.tick();
  };

  terminal::ProgressBar progress_bar(
      static_cast<int32_t>(filtered_view.size()));

  rosbag::TimeTranslator time_translator;

  do {
    time_translator.setRealStartTime(initial_time);
    const ros::WallTime now_wt = ros::WallTime::now();
    time_translator.setTranslatedStartTime(ros::Time(now_wt.sec, now_wt.nsec));

    for (const auto &m : filtered_view) {
      const ros::Time translated = time_translator.translate(m.getTime());
      ros::WallTime horizon = ros::WallTime(translated.sec, translated.nsec);

      bool can_tick = evaluate_tick();

      if (!play_options.is_quiet) {
        std::stringstream prefixs;
        if (can_tick) {
          prefixs << "[Playing] ";
        } else {
          prefixs << "[Paused]  ";
        }
        progress_bar.display(prefixs.str());
      }

      boost::optional<ros::WallTime> paused_time;

      while (!can_tick) {
        if (!paused_time) {
          paused_time.emplace(ros::WallTime::now());
        }

        can_tick = evaluate_tick();
      }

      if (paused_time) {
        ros::WallDuration shift = ros::WallTime::now() - *paused_time;
        time_translator.shift(ros::Duration(shift.sec, shift.nsec));
        horizon += shift;
      }

      ros::WallTime::sleepUntil(horizon);

      publishers[m.getTopic()].publish(m);

      ros::spinOnce();
      ++progress_bar;
    }

    progress_bar.reset();
  } while (play_options.loop);

  std::cout << std::endl << "Done." << std::endl;

  ROS_INFO_STREAM("Requests: " << az_reader.num_requests() << " bytes "
                               << az_reader.num_bytes());
}

} // namespace

int main(int argc, char **argv) {
  CLI::App app("rosbaz - rosbag for azure storage");

  CommonOptions common_options;

  app.add_option(
      "--account-key", common_options.account_key,
      "The storage account key, which you can obtain from the azure portal.");

  app.add_option("--token", common_options.token,
                 "The storage account bearer token, which you can obtain by "
                 "running\naz account "
                 "get-access-token --resource https://storage.azure.com/ -o "
                 "tsv --query accessToken");

  InfoOptions info_options;

  CLI::App *info =
      app.add_subcommand("info", "Summarize the contents of one bag file.")
          ->fallthrough();

  info->add_option("blob_url", info_options.blob_url,
                   "The blob url (may including SAS token)")
      ->required();

  PlayOptions play_options;
  CLI::App *play =
      app.add_subcommand("play", "Play the contents of one bag file.")
          ->fallthrough();
  app.require_subcommand(1);

  play->add_option("blob_url", play_options.blob_url,
                   "The blob url (may including SAS token)")
      ->required();
  play->add_option("-s,--start", play_options.start_offset,
                   "start SEC seconds into the bag files");
  play->add_option("-u,--duration", play_options.duration,
                   "play only SEC seconds from the bag files");
  play->add_flag("-l,--loop", play_options.loop, "loop playback");
  play->add_flag("--pause", play_options.start_paused, "start in paused mode");

  CLI11_PARSE(app, argc, argv);

  for (auto subcom : app.get_subcommands()) {
    if (subcom->get_name() == "info") {
      info_command(common_options, info_options);
    } else if (subcom->get_name() == "play") {
      play_command(common_options, play_options);
    }
  }

  return 0;
}