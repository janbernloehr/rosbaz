# rosbaz

## About

Streaming ros bags from azure blob storage.

## Features

This package provides an implementation of a subset of the rosbag api and the `rosbag` binary allowing streaming ros bags directly from an azure blob storage without prior downloading the bag.

### Example usage

Summarize contents of a bag
```bash
rosbag info https://contosoaccount.blob.core.windows.net/contosocontainer/my.bag?SAS_TOKEN
```

Play back contents of a bag
```bash
rosbag play --paused https://contosoaccount.blob.core.windows.net/contosocontainer/my.bag?SAS_TOKEN
```

Note that only those topics with connected subscribers will actually be downloaded from azure.

### Authentication

The following modes are supported
- Bearer Token
```bash
export AZ_TOKEN=$(az account get-access-token --resource https://storage.azure.com/ -o tsv --query accessToken)
rosbag info https://contosoaccount.blob.core.windows.net/contosocontainer/my.bag --token ???
```
- SAS Token
```bash
rosbag info https://contosoaccount.blob.core.windows.net/contosocontainer/my.bag?SAS_TOKEN
```
- API Key
```bash
rosbag info https://contosoaccount.blob.core.windows.net/contosocontainer/my.bag --token ???
```

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