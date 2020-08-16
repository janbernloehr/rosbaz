# rosbaz

![Catkin CI](https://github.com/janbernloehr/rosbaz/workflows/Catkin%20CI/badge.svg) [![FOSSA Status](https://app.fossa.com/api/projects/git%2Bgithub.com%2Fjanbernloehr%2Frosbaz.svg?type=shield)](https://app.fossa.com/projects/git%2Bgithub.com%2Fjanbernloehr%2Frosbaz?ref=badge_shield)

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
#include <rosbaz/bag.h>
#include <rosbaz/view.h>
#include <std_msgs/Int32.h>

auto url = rosbaz::AzBlobUrl::parse("https://contosoaccount.blob.core.windows.net/contosocontainer/my.bag?SAS_TOKEN");
auto az_reader = std::make_shared<rosbaz::io::AzReader>(url);

auto bag = rosbaz::Bag::read(az_reader);

for(const auto m : rosbag::View(bag))
{
  std_msgs::Int32::ConstPtr i = m.instantiate<std_msgs::Int32>();
  if (i != nullptr) {
    std::cout << i->data << std::endl;
  }
}
```

### Authentication

The following modes are supported
- Bearer Token
```bash
export AZ_TOKEN=$(az account get-access-token --resource https://storage.azure.com/ -o tsv --query accessToken)
rosbaz info https://contosoaccount.blob.core.windows.net/contosocontainer/my.bag --token $AZ_TOKEN
```
- SAS Token
```bash
rosbaz info https://contosoaccount.blob.core.windows.net/contosocontainer/my.bag?SAS_TOKEN
```
- API Key
```bash
rosbaz info https://contosoaccount.blob.core.windows.net/contosocontainer/my.bag --account-key $ACCOUNT_KEY
```

### Restrictions
- rosbag format 2.0
- no support for encryption
- no support for compressed chunks
- only reading (not writing bags) is supported
- only single bag views

## Build (Linux)

### Build azure-storage-cpplite dependency

```bash
git clone --branch v0.3.0 https://github.com/Azure/azure-storage-cpplite.git

cd azure-storage-cpplite
mkdir build.release
cd build.release
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local/azure-storage-cpplite -DBUILD_SHARED_LIBS=ON

cmake --build .
sudo cmake --build . --target install
```

Also add it to your `LD_LIBRARY_PATH`
```bash
echo "export LD_LIBRARY_PATH=/usr/local/azure-storage-cpplite/lib:$LD_LIBRARY_PATH" >> ~/.bashrc
source ~/.bashrc
```

For more details, refer to https://github.com/Azure/azure-storage-cpplite/tree/v0.3.0#installation

### Build rosbaz

```bash
mkdir rosbaz
cd rosbaz
git clone https://github.com/janbernloehr/rosbaz.git src
catkin build rosbaz
```

## Dependencies

- [azure-storage-cpplite](https://github.com/Azure/azure-storage-cpplite)
- [CLI11](https://github.com/CLIUtils/CLI11)
- [span-lite](https://github.com/martinmoene/span-lite)
