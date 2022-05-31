#include "rosbaz/io/az_writer.h"

#ifndef NO_AZ_BINDINGS

#include "az_bearer_token.h"

#include <azure/core.hpp>
#include <azure/storage/blobs.hpp>

#include "rosbaz/exceptions.h"
#include "rosbaz/io/io_helpers.h"
#include "rosbaz/io/util.h"

#include <numeric>
#include <iostream>

namespace rosbaz
{
namespace io
{
AzBlock::AzBlock(AzWriter& writer, size_t block_offset) : Block{ block_offset }, writer_{ writer }
{
  const size_t id = writer.blocks_.size();
  std::vector<uint8_t> id_as(sizeof(size_t));
  std::copy_n(reinterpret_cast<const uint8_t*>(&id), sizeof(size_t), id_as.data());
  id_ = Azure::Core::Convert::Base64Encode(id_as);
}

void AzBlock::write(rosbaz::io::byte const* data, size_t n, boost::optional<size_t> offset)
{
  if (is_staged())
  {
    throw BlockStagedException("Cannot write to staged block");
  }
  size_t new_size = buffer_.size();

  if (offset)
  {
    new_size = std::max(new_size, *offset + n);
  }
  else
  {
    new_size += n;
  }

  const size_t original_size = buffer_.size();

  buffer_.resize(new_size);
  rosbaz::io::byte* output = buffer_.data() + offset.value_or(original_size);

  std::copy_n(data, n, output);
}

size_t AzBlock::size() const
{
  return buffer_.size();
}

void AzBlock::stage()
{
  Azure::Core::IO::MemoryBodyStream memory_stream{ buffer_.data(), buffer_.size() };
  writer_.client_->StageBlock(id_, memory_stream);
  is_staged_ = true;
}

AzWriter::AzWriter(const AzBlobUrl& blob_url, const std::string& account_key, const std::string& token)
  : container_(blob_url.container_name), blob_(blob_url.blob_name)
{
  std::shared_ptr<Azure::Storage::Blobs::BlockBlobClient> blobClient;

  if (!token.empty())
  {
    auto credential = std::make_shared<BearerToken>(token);
    client_ = std::make_shared<Azure::Storage::Blobs::BlockBlobClient>(blob_url.to_string(), credential);
  }
  else if (!blob_url.sas_token.empty())
  {
    client_ = std::make_shared<Azure::Storage::Blobs::BlockBlobClient>(blob_url.to_string());
  }
  else if (!account_key.empty())
  {
    auto credential = std::make_shared<Azure::Storage::StorageSharedKeyCredential>(blob_url.account_name, account_key);
    client_ = std::make_shared<Azure::Storage::Blobs::BlockBlobClient>(blob_url.to_string(), credential);
  }
  else
  {
    throw rosbaz::MissingCredentialsException("You must provide either a bearer token, a sas "
                                              "token, or an account key.");
  }
}

AzWriter::~AzWriter() = default;

std::string AzWriter::filepath()
{
  return client_->GetUrl();
}

size_t AzWriter::size()
{
  std::lock_guard<std::mutex> lock_guard(mutex_);

  return std::accumulate(blocks_.begin(), blocks_.end(), 0,
                         [](size_t a, const auto& block) { return block->is_staged() ? a + block->size() : a; });
}

std::shared_ptr<Block> AzWriter::create_block()
{
  size_t position = size();
  std::lock_guard<std::mutex> lock_guard(mutex_);
  if (has_unstaged_blocks())
  {
    throw UnstagedBlocksException("Cannot create a new block while unstaged blocks present");
  }

  auto block = std::make_shared<AzBlock>(*this, position);
  blocks_.push_back(block);
  return block;
}

std::shared_ptr<Block> AzWriter::replace_block(Block& original)
{
  std::lock_guard<std::mutex> lock_guard(mutex_);
  if (has_unstaged_blocks())
  {
    throw UnstagedBlocksException("Cannot create a new block while unstaged blocks present");
  }

  auto found =
      std::find_if(blocks_.begin(), blocks_.end(), [&original](const auto& block) { return &*block == &original; });

  if (found == blocks_.end())
  {
    throw Exception("Could not find given block in list");
  }

  *found = std::make_shared<AzBlock>(*this, original.block_offset());
  return *found;
}

void AzWriter::commit_blocks()
{
  std::vector<std::string> block_ids;
  for (const auto& block : blocks_)
  {
    block_ids.push_back(block->id());
  }

  client_->CommitBlockList(block_ids);
}

bool AzWriter::has_unstaged_blocks() const
{
  return std::any_of(blocks_.begin(), blocks_.end(), [](const auto& block) { return !block->is_staged(); });
}

}  // namespace io
}  // namespace rosbaz
#endif
