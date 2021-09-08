#include "rosbaz/app/bag_printer.h"

#include "rosbaz/bag_statistics.h"
#include <boost/format.hpp>

namespace
{
std::string human_readable_size(size_t size)
{
  if (size > 1024 * 1024 * 1024)
  {
    return std::to_string(static_cast<float>(size) / 1024.f / 1024.f / 1024.f) + " GB";
  }
  else if (size > 1024 * 1024)
  {
    return std::to_string(static_cast<float>(size) / 1024.f / 1024.f) + " MB";
  }
  else
  {
    return std::to_string(static_cast<float>(size) / 1024.f) + " KB";
  }
}
}  // namespace

namespace rosbaz
{
namespace app
{
void write_bag_human_friendly(const rosbaz::Bag& bag, std::ostream& stream)
{
  auto start_time = bag.getBeginTime();
  auto end_time = bag.getEndTime();
  auto duration = end_time - start_time;

  stream << "path:     " << bag.getFilePath() << "\n";
  stream << "version:  " << bag.getMajorVersion() << "." << bag.getMinorVersion() << "\n";
  stream << "duration: " << duration.toSec() << "s\n";
  std::ios_base::fmtflags f(stream.flags());
  stream << "start:    " << std::fixed << std::setprecision(6) << start_time.toSec() << "\n";
  stream << "end:      " << end_time.toSec() << "\n";
  stream.flags(f);

  stream << "size:     " << human_readable_size(bag.getSize()) << "\n";

  rosbaz::BagStatistics stats{ bag };

  stream << "messages: " << stats.getTotalMessageCount() << "\n";
  stream << "chunks:   " << bag.getChunkCount() << "\n";

  const auto msg_type_infos = stats.getMessageTypeInfos();
  const auto max_datatype =
      std::max_element(msg_type_infos.begin(), msg_type_infos.end(),
                       [](const auto& a, const auto& b) { return a.datatype.size() < b.datatype.size(); });

  boost::format type_fmt("%s %-" + std::to_string(max_datatype->datatype.size()) + "s [%s]\n");

  std::string prefix = "types:   ";
  for (const auto& msg_type_info : msg_type_infos)
  {
    stream << boost::str(type_fmt % prefix % msg_type_info.datatype % msg_type_info.md5sum);
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
    return a.frequency.value_or(-1.) < b.frequency.value_or(-1.);
  });

  const size_t max_topic = max_topic_it != topic_infos.end() ? max_topic_it->topic.size() : 0;
  const size_t max_msgs = max_msgs_it != topic_infos.end() ? max_msgs_it->num_messages : 0;
  const double max_freq = max_freq_it != topic_infos.end() ? max_freq_it->frequency.value_or(0.) : 0.;

  std::string topic_fmt_string = "%s %-" + std::to_string(max_topic) + "s  %" +
                                 std::to_string(std::to_string(max_msgs).size()) + "s msgs @ %" +
                                 std::to_string((boost::format("%.1f") % max_freq).size()) + ".1f %s : %s\n";
  std::string topic_fmt_no_freq_string = "%s %-" + std::to_string(max_topic) + "s  %" +
                                         std::to_string(std::to_string(max_msgs).size()) + "s msgs    : %s\n";

  boost::format topic_fmt(topic_fmt_string);
  boost::format topic_fmt_no_freq(topic_fmt_no_freq_string);

  prefix = "topics:  ";
  for (const auto& topic_info : topic_infos)
  {
    if (topic_info.frequency)
    {
      stream << boost::str(topic_fmt % prefix % topic_info.topic % topic_info.num_messages % *topic_info.frequency %
                           "hz" % topic_info.datatype);
    }
    else
    {
      stream << boost::str(topic_fmt_no_freq % prefix % topic_info.topic % topic_info.num_messages %
                           topic_info.datatype);
    }
    prefix = "         ";
  }
}

void write_bag_yaml(const rosbaz::Bag& bag, std::ostream& stream)
{
  auto start_time = bag.getBeginTime();
  auto end_time = bag.getEndTime();
  auto duration = end_time - start_time;

  stream << "path: " << bag.getFilePath() << "\n";
  stream << "version: " << bag.getMajorVersion() << "." << bag.getMinorVersion() << "\n";
  stream << "duration: " << duration.toSec() << "\n";
  std::ios_base::fmtflags f(stream.flags());
  stream << "start: " << std::fixed << std::setprecision(6) << start_time.toSec() << "\n";
  stream << "end: " << end_time.toSec() << "\n";
  stream.flags(f);
  stream << "size: " << bag.getSize() << "\n";

  rosbaz::BagStatistics stats{ bag };

  stream << "messages: " << stats.getTotalMessageCount() << "\n";
  stream << "indexed: "
         << "True"
         << "\n";
  stream << "compression: "
         << "none"
         << "\n";

  stream << "types:\n";

  const auto msg_type_infos = stats.getMessageTypeInfos();

  for (const auto& msg_type_info : msg_type_infos)
  {
    stream << "    - type: " << msg_type_info.datatype << "\n";
    stream << "      md5: " << msg_type_info.md5sum << "\n";
  }

  stream << "topics:\n";

  const auto topic_infos = stats.getMessageTopicInfos();

  for (const auto& topic_info : topic_infos)
  {
    stream << "    - topic: " << topic_info.topic << "\n";
    stream << "      type: " << topic_info.datatype << "\n";
    stream << "      messages: " << topic_info.num_messages << "\n";

    if (topic_info.frequency)
    {
      stream << "      frequency: " << *topic_info.frequency << "\n";
    }
  }
}
}  // namespace app
}  // namespace rosbaz