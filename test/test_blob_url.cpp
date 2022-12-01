#include "rosbaz/blob_url.h"

#include <gtest/gtest.h>
#include <string>

TEST(AzBlobUrl, can_parse_without_sas)
{
  const std::string url = "https://contosoaccount.blob.core.windows.net/contosocontainer/my.bag";

  const auto az_url = rosbaz::AzBlobUrl::parse(url);

  EXPECT_EQ(az_url.schema, "https");
  EXPECT_EQ(az_url.account_name, "contosoaccount");
  EXPECT_EQ(az_url.blob_endpoint, "contosoaccount.blob.core.windows.net");
  EXPECT_EQ(az_url.container_name, "contosocontainer");
  EXPECT_EQ(az_url.blob_name, "my.bag");
  EXPECT_EQ(az_url.sas_token, "");
}

TEST(AzBlobUrl, can_parse_with_sas)
{
  const std::string url = "https://contosoaccount.blob.core.windows.net/"
                          "contosocontainer/my.bag?SAS_TOKEN";

  const auto az_url = rosbaz::AzBlobUrl::parse(url);

  EXPECT_EQ(az_url.schema, "https");
  EXPECT_EQ(az_url.account_name, "contosoaccount");
  EXPECT_EQ(az_url.blob_endpoint, "contosoaccount.blob.core.windows.net");
  EXPECT_EQ(az_url.container_name, "contosocontainer");
  EXPECT_EQ(az_url.blob_name, "my.bag");
  EXPECT_EQ(az_url.sas_token, "?SAS_TOKEN");
}
