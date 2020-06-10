
#include "gtest/gtest.h"

#include "rosbaz/io/io_helpers.h"

TEST(AzBlobUrl, to_string) {
  const std::string expected = "lorem ipsum dolor sit amet";

  const rosbaz::DataSpan span(
      reinterpret_cast<const uint8_t *>(expected.c_str()), expected.size());

  const std::string actual = rosbaz::io::to_string(span);

  EXPECT_EQ(expected, actual);
}

TEST(AzBlobUrl, read_little_endian_uint32_t) {
  std::array<uint8_t, 4> data{71, 71, 71, 71};
  const uint32_t expected = 21;

  std::copy(&expected, &expected + 1,
            reinterpret_cast<uint32_t *>(data.begin()));

  const uint32_t actual = rosbaz::io::read_little_endian<uint32_t>(data);

  EXPECT_EQ(expected, actual);
}
