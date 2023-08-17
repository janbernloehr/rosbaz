# rosbaz

![Catkin CI](https://github.com/janbernloehr/rosbaz/workflows/Catkin%20CI/badge.svg) [![Codacy Badge](https://api.codacy.com/project/badge/Grade/3338150d8ef54a1d949264825e84686f)](https://app.codacy.com/manual/janbernloehr/rosbaz?utm_source=github.com&utm_medium=referral&utm_content=janbernloehr/rosbaz&utm_campaign=Badge_Grade_Dashboard) [![FOSSA Status](https://app.fossa.com/api/projects/git%2Bgithub.com%2Fjanbernloehr%2Frosbaz.svg?type=shield)](https://app.fossa.com/projects/git%2Bgithub.com%2Fjanbernloehr%2Frosbaz?ref=badge_shield)

## About

Streaming ros bags from azure blob storage.

## Features

This package provides an implementation of a subset of the rosbag api and the `rosbag` binary allowing streaming ros bags directly from an azure blob storage without prior downloading the bag.

### Example usage

Summarize contents of a bag

```bash
rosbaz info https://contosoaccount.blob.core.windows.net/contosocontainer/my.bag?SAS_TOKEN
```

Play back contents of a bag

```bash
rosbaz play --paused https://contosoaccount.blob.core.windows.net/contosocontainer/my.bag?SAS_TOKEN
```

Note that only those topics with connected subscribers will actually be downloaded from azure.

We tried to keep the API as close to the rosbag api as possible.

```c++
#include "rosbaz/io/az_reader.h"
#include "rosbaz/io/az_writer.h

#include <rosbaz/bag.h>
#include <rosbaz/view.h>
#include <std_msgs/Int32.h>

auto in_url = rosbaz::AzBlobUrl::parse("https://contosoaccount.blob.core.windows.net/contosocontainer/my.bag?SAS_TOKEN");
auto az_reader = std::make_shared<rosbaz::io::AzReader>(in_url);
auto in_bag = rosbaz::Bag::read(az_reader);

for(const auto m : rosbaz::View(in_bag))
{
  std_msgs::Int32::ConstPtr i = m.instantiate<std_msgs::Int32>();
  if (i != nullptr) {
    std::cout << i->data << std::endl;
  }
}

auto out_url = rosbaz::AzBlobUrl::parse("https://contosoaccount.blob.core.windows.net/contosocontainer/other.bag?SAS_TOKEN");

auto az_writer = std::make_shared<rosbaz::io::AzWriter>(out_url);
auto out_bag = rosbaz::Bag::write(az_writer);

for(const auto m : rosbaz::View(in_bag))
{
  out_bag.write(m.getTopic(), m.getTime(), m);
}

out_bag.close();
```

### Authentication

The following modes are supported

-   Azure Cli Credential

```bash
az login

rosbaz info https://contosoaccount.blob.core.windows.net/contosocontainer/my.bag
```

-   Bearer Token

```bash
export AZ_TOKEN=$(az account get-access-token --resource https://storage.azure.com/ -o tsv --query accessToken)
rosbaz info https://contosoaccount.blob.core.windows.net/contosocontainer/my.bag --token $AZ_TOKEN
```

-   SAS Token

```bash
rosbaz info https://contosoaccount.blob.core.windows.net/contosocontainer/my.bag?SAS_TOKEN
```

-   API Key

```bash
rosbaz info https://contosoaccount.blob.core.windows.net/contosocontainer/my.bag --account-key $ACCOUNT_KEY
```

### Restrictions

-   rosbag format 2.0
-   no support for encryption
-   no support for compressed chunks

## Build (Linux)

### Install dependencies

1. Install prerequsites
```bash
sudo apt install -y libssl-dev libcurl4-openssl-dev cmake g++ uuid-dev libxml2-dev
```

2. Make sure you have a sufficiently recent cmake, you can install the latest version by
```bash
sudo apt install software-properties-common

curl -s https://apt.kitware.com/keys/kitware-archive-latest.asc | sudo apt-key add -
sudo apt-add-repository "deb https://apt.kitware.com/ubuntu/ $(lsb_release -sc) main"

sudo apt install cmake
```

3. Install azure-storage-blobs
```bash
git clone --branch azure-storage-blobs_12.8.0 https://github.com/Azure/azure-sdk-for-cpp.git

cd azure-sdk-for-cpp
mkdir build.release
cd build.release
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON

cmake --build . --parallel
sudo cmake --install .
```

### Build rosbaz

```bash
mkdir rosbaz
cd rosbaz
git clone https://github.com/janbernloehr/rosbaz.git src
catkin build rosbaz
```

## Dependencies

-   [azure-sdk-for-cpp](https://github.com/Azure/azure-sdk-for-cpp.git)
-   [CLI11](https://github.com/CLIUtils/CLI11)
-   [span-lite](https://github.com/martinmoene/span-lite)

## License

[![FOSSA Status](https://app.fossa.com/api/projects/git%2Bgithub.com%2Fjanbernloehr%2Frosbaz.svg?type=large)](https://app.fossa.com/projects/git%2Bgithub.com%2Fjanbernloehr%2Frosbaz?ref=badge_large)
