#include "gtest/gtest.h"

#include "rosbaz/bag.h"
#include "rosbaz/io/io_helpers.h"
#include "rosbaz/io/stream_reader.h"
#include "rosbaz/io/stream_writer.h"
#include "rosbaz/view.h"

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>

#include <boost/filesystem.hpp>
#include <memory>

class WriteTests : public ::testing::TestWithParam<const char*>
{
public:
  WriteTests() : bag{ std::make_unique<rosbag::Bag>(GetParam()) }
  {
    const boost::filesystem::path output_path = boost::filesystem::temp_directory_path() / (boost::filesystem::unique_path().string() + ".bag");
    rosbaz::Bag output_bag = rosbaz::Bag::write(rosbaz::io::StreamWriter::open(output_path.native()));

    rosbag::View view_in(*bag);
    for (const auto& m : view_in)
    {
      output_bag.write(m.getTopic(), m.getTime(), m);
    }

    output_bag.close();

    baz = std::make_unique<rosbaz::Bag>(rosbaz::Bag::read(rosbaz::io::StreamReader::open(output_path.native())));
  }

  std::unique_ptr<rosbaz::Bag> baz;
  std::unique_ptr<rosbag::Bag> bag;
};

TEST_P(WriteTests, equal_properties)
{
  ASSERT_EQ(baz->getMode(), bag->getMode());
  ASSERT_EQ(baz->getMajorVersion(), bag->getMajorVersion());
  ASSERT_EQ(baz->getMinorVersion(), bag->getMinorVersion());
}

TEST_P(WriteTests, equal_topics)
{
  rosbaz::View baz_view{ *baz };
  rosbag::View bag_view{ *bag };

  const auto& bag_connections = bag_view.getConnections();
  const auto& baz_connections = baz_view.getConnections();

  ASSERT_EQ(bag_connections.size(), baz_connections.size());

  for (size_t i = 0; i < bag_connections.size(); i++)
  {
    const auto* bag_connection = bag_connections[i];
    const auto found_connection = std::find_if(baz_connections.begin(), baz_connections.end(),
                                               [bag_connection](const rosbag::ConnectionInfo* other_connection) {
                                                 return other_connection->id == bag_connection->id;
                                               });

    ASSERT_NE(found_connection, baz_connections.end());
    const auto* baz_connection = *found_connection;

    EXPECT_EQ(bag_connection->topic, baz_connection->topic);
    EXPECT_EQ(bag_connection->datatype, baz_connection->datatype);
    EXPECT_EQ(bag_connection->md5sum, baz_connection->md5sum);
    EXPECT_EQ(bag_connection->msg_def, baz_connection->msg_def);
  }
}

TEST_P(WriteTests, equal_messages)
{
  const std::string topic_name = "imu";

  rosbaz::View baz_view{ *baz, rosbag::TopicQuery(topic_name) };
  rosbag::View bag_view{ *bag, rosbag::TopicQuery(topic_name) };

  ASSERT_EQ(baz_view.size(), bag_view.size());
  ASSERT_EQ(baz_view.getBeginTime(), bag_view.getBeginTime());
  ASSERT_EQ(baz_view.getEndTime(), bag_view.getEndTime());

  for (size_t i = 0; i < baz_view.size(); ++i)
  {
    auto baz_m = baz_view.begin();
    std::advance(baz_m, i);
    auto baz_message = baz_m->instantiate<sensor_msgs::Imu>();

    auto bag_m = bag_view.begin();
    std::advance(bag_m, i);
    auto bag_message = bag_m->instantiate<sensor_msgs::Imu>();

    EXPECT_EQ(bag_m->getTime(), baz_m->getTime());
    EXPECT_EQ(bag_m->getTopic(), baz_m->getTopic());
    EXPECT_EQ(bag_m->getDataType(), baz_m->getDataType());
    EXPECT_EQ(bag_m->getMD5Sum(), baz_m->getMD5Sum());
    EXPECT_EQ(bag_m->getMessageDefinition(), baz_m->getMessageDefinition());

    EXPECT_EQ(bag_message->header.seq, baz_message->header.seq);
  }
}

INSTANTIATE_TEST_CASE_P(WriteTestSuite, WriteTests, testing::Values("b0-2014-07-11-10-58-16-decompressed.bag"));
