#include <regex>

#include <boost/optional.hpp>
#include <boost/optional/optional_io.hpp>

#include "cli11/CLI11.hpp"

#include <ros/console.h>
#include <ros/ros.h>
#include <rosbag/time_translator.h>

#include "terminal/pausable_context.h"
#include "terminal/progress_bar.h"

#include "rosbaz/bag.h"
#include "rosbaz/blob_url.h"
#include "rosbaz/io/az_reader.h"
#include "rosbaz/io/stream_reader.h"
#include "rosbaz/view.h"
#include "rosbaz/bag_statistics.h"

namespace container
{
template <class T>
inline std::istream& operator>>(std::istream& stream, boost::optional<T>& optional)
{
  optional.emplace();
  stream >> optional.value();
  return stream;
}

}  // namespace container

struct CommonOptions
{
  std::string account_key{};
  std::string token{};

  bool print_transfer_stats{ false };
};

struct InfoOptions
{
  std::string file_or_blob_url{};

  bool topic_message_frequency_statistics = false;
  bool yaml_output = false;
};

struct PlayOptions
{
  std::string file_or_blob_url{};

  boost::optional<float> start_offset{};
  boost::optional<float> duration{};

  bool loop = false;
  bool start_paused = false;
  bool is_quiet = false;
};

namespace
{
std::regex is_url_rgx("^(https*)://");

bool is_url(const std::string& path_or_url)
{
  std::smatch matches;
  return std::regex_search(path_or_url, matches, is_url_rgx);
}

void print_bag(const rosbaz::Bag& bag)
{
  auto start_time = bag.getBeginTime();
  auto end_time = bag.getEndTime();
  auto duration = end_time - start_time;

  std::cout << "path:     " << bag.getFilePath() << "\n";
  std::cout << "version:  " << bag.getMajorVersion() << "." << bag.getMinorVersion() << "\n";
  std::cout << "duration: " << duration.toSec() << "s\n";
  std::cout << "start:    " << start_time.sec << "." << start_time.nsec << "\n";
  std::cout << "end:      " << end_time.sec << "." << end_time.nsec << "\n";
  std::cout << "size:     " << static_cast<float>(bag.getSize()) / 1024.f / 1024.f / 1024.f << " GB\n";

  rosbaz::BagSatistics stats{ bag };

  std::cout << "messages: " << stats.getTotalMessageCount() << "\n";
  std::cout << "chunks:   " << bag.getChunkCount() << "\n";

  const auto msg_type_infos = stats.getMessageTypeInfos();
  const auto max_datatype =
      std::max_element(msg_type_infos.begin(), msg_type_infos.end(),
                       [](const auto& a, const auto& b) { return a.datatype.size() < b.datatype.size(); });

  boost::format type_fmt("%s %-" + std::to_string(max_datatype->datatype.size()) + "s [%s]\n");

  std::string prefix = "types:   ";
  for (const auto& msg_type_info : msg_type_infos)
  {
    std::cout << boost::str(type_fmt % prefix % msg_type_info.datatype % msg_type_info.md5sum);
    prefix = "         ";
  }

  const auto topic_infos = stats.getMessageTopicInfos();
  const auto max_topic_it = std::max_element(topic_infos.begin(), topic_infos.end(), [](const auto& a, const auto& b) {
    return a.topic.size() < b.topic.size();
  });
  const auto max_msgs_it = std::max_element(topic_infos.begin(), topic_infos.end(), [](const auto& a, const auto& b) {
    return a.num_messages < b.num_messages;
  });
  const auto max_freq_it = std::max_element(topic_infos.begin(), topic_infos.end(), [](const auto& a, const auto& b) {
    return (a.frequency && b.frequency) && a.frequency < b.frequency;
  });

  const size_t max_topic = max_topic_it != topic_infos.end() ? max_topic_it->topic.size() : 0;
  const size_t max_msgs = max_msgs_it != topic_infos.end() ? max_msgs_it->num_messages : 0;
  const size_t max_freq =
      max_freq_it != topic_infos.end() ? static_cast<size_t>(max_freq_it->frequency.value_or(0.)) : 0;

  std::string topic_fmt_string = "%s %-" + std::to_string(max_topic) + "s  %" +
                                 std::to_string(std::to_string(max_msgs).size()) + "s msgs @ %" +
                                 std::to_string(std::to_string(max_freq).size() + 2) + ".1f %s : %s\n";
  std::string topic_fmt_no_freq_string = "%s %-" + std::to_string(max_topic) + "s  %" +
                                         std::to_string(std::to_string(max_msgs).size()) + "s msgs    : %s\n";

  boost::format topic_fmt(topic_fmt_string);
  boost::format topic_fmt_no_freq(topic_fmt_no_freq_string);

  prefix = "topics:  ";
  for (const auto& topic_info : topic_infos)
  {
    if (topic_info.frequency)
    {
      std::cout << boost::str(topic_fmt % prefix % topic_info.topic % topic_info.num_messages % *topic_info.frequency %
                              "hz" % topic_info.datatype);
    }
    else
    {
      std::cout << boost::str(topic_fmt_no_freq % prefix % topic_info.topic % topic_info.num_messages %
                              topic_info.datatype);
    }
    prefix = "         ";
  }
}

void print_bag_yaml(const rosbaz::Bag& bag)
{
  auto start_time = bag.getBeginTime();
  auto end_time = bag.getEndTime();
  auto duration = end_time - start_time;

  std::cout << "path: " << bag.getFilePath() << "\n";
  std::cout << "version: " << bag.getMajorVersion() << "." << bag.getMinorVersion() << "\n";
  std::cout << "duration: " << duration.toSec() << "s\n";
  std::cout << "start: " << start_time.sec << "." << start_time.nsec << "\n";
  std::cout << "end: " << end_time.sec << "." << end_time.nsec << "\n";
  std::cout << "size: " << bag.getSize() << "\n";

  rosbaz::BagSatistics stats{ bag };

  std::cout << "messages: " << stats.getTotalMessageCount() << "\n";
  std::cout << "indexed: "
            << "True"
            << "\n";
  std::cout << "compression: "
            << "none"
            << "\n";

  std::cout << "types:\n";

  const auto msg_type_infos = stats.getMessageTypeInfos();

  for (const auto& msg_type_info : msg_type_infos)
  {
    std::cout << "    - type: " << msg_type_info.datatype << "\n";
    std::cout << "      md5: " << msg_type_info.md5sum << "\n";
  }

  std::cout << "topics:\n";
  
  const auto topic_infos = stats.getMessageTopicInfos();

  for (const auto& topic_info : topic_infos)
  {
    std::cout << "    - topic: " << topic_info.topic << "\n";
    std::cout << "      type: " << topic_info.datatype << "\n";
    std::cout << "      messages: " << topic_info.num_messages << "\n";

    if (topic_info.frequency)
    {
      std::cout << "      frequency: " << *topic_info.frequency << "\n";
    }
  }
}

void print_transfer_stats(const rosbaz::io::IReader& reader)
{
  if (auto az_reader = dynamic_cast<const rosbaz::io::AzReader*>(&reader))
  {
    std::cout << "---\n";
    std::cout << "requests:          " << az_reader->num_requests() << "\n";
    std::cout << "bytes transferred: " << az_reader->num_bytes() << "\n";
  }
}

std::shared_ptr<rosbaz::io::IReader> create_reader(const std::string& path_or_url, const std::string& account_key,
                                                   const std::string& token)
{
  if (is_url(path_or_url))
  {
    auto url = rosbaz::AzBlobUrl::parse(path_or_url);

    return std::make_shared<rosbaz::io::AzReader>(url, account_key, token);
  }
  else
  {
    return rosbaz::io::StreamReader::open(path_or_url);
  }
}

void info_command(const CommonOptions& common_options, const InfoOptions& info_options)
{
  auto az_reader = create_reader(info_options.file_or_blob_url, common_options.account_key, common_options.token);
  auto az_bag = rosbaz::Bag::read(az_reader, info_options.topic_message_frequency_statistics);

  if (info_options.yaml_output)
  {
    print_bag_yaml(az_bag);
  }
  else
  {
    print_bag(az_bag);
  }

  if (common_options.print_transfer_stats)
  {
    print_transfer_stats(*az_reader);
  }
}

void play_command(const CommonOptions& common_options, const PlayOptions& play_options)
{
  // setup node handle
  std::map<std::string, std::string> mapping{};
  ros::init(mapping, "rosbaz");
  ros::NodeHandle node_handle{};

  auto az_reader = create_reader(play_options.file_or_blob_url, common_options.account_key, common_options.token);
  auto az_bag = rosbaz::Bag::read(az_reader);

  // we first create a full blown view to obtain the time range of the messages
  // and then filter that view
  rosbaz::View full_view{ az_bag };

  const auto initial_time = full_view.getBeginTime() + ros::Duration(play_options.start_offset.value_or(0));
  ros::Time finish_time = ros::TIME_MAX;
  if (play_options.duration)
  {
    finish_time = initial_time + ros::Duration(*play_options.duration);
  }

  rosbaz::View filtered_view{ az_bag, initial_time, finish_time };

  std::unordered_map<std::string, ros::Publisher> publishers;
  for (const auto* connection_info : filtered_view.getConnections())
  {
    ros::AdvertiseOptions options(connection_info->topic, 50, connection_info->md5sum, connection_info->datatype,
                                  connection_info->msg_def);
    publishers[connection_info->topic] = node_handle.advertise(options);
  }

  terminal::PausableContext pausable_context{ play_options.start_paused };

  auto evaluate_tick = [&pausable_context]() { return pausable_context.tick(); };

  terminal::ProgressBar progress_bar(static_cast<int32_t>(filtered_view.size()));

  rosbag::TimeTranslator time_translator;

  do
  {
    time_translator.setRealStartTime(initial_time);
    const ros::WallTime now_wt = ros::WallTime::now();
    time_translator.setTranslatedStartTime(ros::Time(now_wt.sec, now_wt.nsec));

    for (const auto& m : filtered_view)
    {
      const ros::Time translated = time_translator.translate(m.getTime());
      ros::WallTime horizon = ros::WallTime(translated.sec, translated.nsec);

      bool can_tick = evaluate_tick();

      if (!play_options.is_quiet)
      {
        std::stringstream prefixs;
        if (can_tick)
        {
          prefixs << "[Playing] ";
        }
        else
        {
          prefixs << "[Paused]  ";
        }
        progress_bar.display(prefixs.str());
      }

      boost::optional<ros::WallTime> paused_time;

      while (!can_tick)
      {
        if (!paused_time)
        {
          paused_time.emplace(ros::WallTime::now());
        }

        can_tick = evaluate_tick();
      }

      if (paused_time)
      {
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

  if (common_options.print_transfer_stats)
  {
    print_transfer_stats(*az_reader);
  }
}

}  // namespace

int main(int argc, char** argv)
{
  CLI::App app("rosbaz - rosbag for azure storage");

  CommonOptions common_options;

  app.add_option("--account-key", common_options.account_key,
                 "The storage account key, which you can obtain from the azure portal.");

  app.add_option("--token", common_options.token,
                 "The storage account bearer token, which you can obtain by "
                 "running\naz account "
                 "get-access-token --resource https://storage.azure.com/ -o "
                 "tsv --query accessToken");

  app.add_flag("--print-transfer-stats", common_options.print_transfer_stats,
               "Print transfer statistics at command completion.");

  InfoOptions info_options;

  CLI::App* info = app.add_subcommand("info", "Summarize the contents of one bag file.")->fallthrough();

  info->add_option("file_or_blob_url", info_options.file_or_blob_url,
                   "Either a path to a file or a blob url (may including SAS token)")
      ->required();
  info->add_flag("--freq", info_options.topic_message_frequency_statistics,
                 "display topic message frequency statistics");
  info->add_flag("-y,--yaml", info_options.yaml_output, "print information in YAML format");

  PlayOptions play_options;
  CLI::App* play = app.add_subcommand("play", "Play the contents of one bag file.")->fallthrough();
  app.require_subcommand(1);

  play->add_option("file_or_blob_url", play_options.file_or_blob_url,
                   "Either a path to a file or a blob url (may including SAS token)")
      ->required();
  play->add_option("-s,--start", play_options.start_offset, "start SEC seconds into the bag files");
  play->add_option("-u,--duration", play_options.duration, "play only SEC seconds from the bag files");
  play->add_flag("-l,--loop", play_options.loop, "loop playback");
  play->add_flag("--pause", play_options.start_paused, "start in paused mode");

  CLI11_PARSE(app, argc, argv);

  for (auto subcom : app.get_subcommands())
  {
    if (subcom->get_name() == "info")
    {
      info_command(common_options, info_options);
    }
    else if (subcom->get_name() == "play")
    {
      play_command(common_options, play_options);
    }
  }

  return 0;
}