#include <regex>

#include <boost/optional.hpp>
#include <boost/optional/optional_io.hpp>

#include "cli11/CLI11.hpp"

#include <ros/console.h>
#include <ros/ros.h>
#include <rosbag/time_translator.h>

#include "terminal/pausable_context.h"
#include "terminal/progress_bar.h"

#include "rosbaz/app/bag_printer.h"
#include "rosbaz/app/options.h"
#include "rosbaz/bag.h"
#include "rosbaz/blob_url.h"
#include "rosbaz/io/az_reader.h"
#include "rosbaz/io/stream_reader.h"
#include "rosbaz/view.h"

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

namespace
{
void print_bag(const rosbaz::Bag& bag)
{
  rosbaz::app::write_bag_human_friendly(bag, std::cout);
}

void print_bag_yaml(const rosbaz::Bag& bag)
{
  rosbaz::app::write_bag_yaml(bag, std::cout);
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
  if (rosbaz::is_url(path_or_url))
  {
    auto url = rosbaz::AzBlobUrl::parse(path_or_url);

    return std::make_shared<rosbaz::io::AzReader>(url, account_key, token);
  }
  else
  {
    return rosbaz::io::StreamReader::open(path_or_url);
  }
}

void info_command(const rosbaz::app::CommonOptions& common_options, const rosbaz::app::InfoOptions& info_options)
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

void play_command(const rosbaz::app::CommonOptions& common_options, const rosbaz::app::PlayOptions& play_options)
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

  rosbaz::app::CommonOptions common_options;

  app.add_option("--account-key", common_options.account_key,
                 "The storage account key, which you can obtain from the azure portal.");

  app.add_option("--token", common_options.token,
                 "The storage account bearer token, which you can obtain by "
                 "running\naz account "
                 "get-access-token --resource https://storage.azure.com/ -o "
                 "tsv --query accessToken");

  app.add_flag("--print-transfer-stats", common_options.print_transfer_stats,
               "Print transfer statistics at command completion.");

  rosbaz::app::InfoOptions info_options;

  CLI::App* info = app.add_subcommand("info", "Summarize the contents of one bag file.")->fallthrough();

  info->add_option("file_or_blob_url", info_options.file_or_blob_url,
                   "Either a path to a file or a blob url (may including SAS token)")
      ->required();
  info->add_flag("--freq", info_options.topic_message_frequency_statistics,
                 "display topic message frequency statistics");
  info->add_flag("-y,--yaml", info_options.yaml_output, "print information in YAML format");

  rosbaz::app::PlayOptions play_options;
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