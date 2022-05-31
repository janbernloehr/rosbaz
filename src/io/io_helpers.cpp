#include "rosbaz/io/io_helpers.h"

#include <array>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "rosbaz/common.h"
#include "rosbaz/exceptions.h"

namespace rosbaz
{
namespace io
{
void read_from(RosStream& ifs, byte* target, const size_t size)
{
  assert(size <= static_cast<size_t>(std::numeric_limits<std::streamsize>::max()));
  const auto ssize = static_cast<std::streamsize>(size);

  ifs.read(reinterpret_cast<char*>(target), ssize);

  const auto actual_size = ifs.gcount();

  if (actual_size != ssize)
  {
    std::stringstream msg;
    msg << "Requested " << ssize << " from stream but " << actual_size << " were read";
    throw rosbaz::IoException(msg.str());
  }
}

}  // namespace io

}  // namespace rosbaz
