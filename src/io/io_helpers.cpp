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
void read_from(RosStream& ifs, byte* target, const size_t count)
{
  assert(count <= static_cast<size_t>(std::numeric_limits<std::streamsize>::max()));
  const auto scount = static_cast<std::streamsize>(count);

  ifs.read(reinterpret_cast<char*>(target), scount);

  const auto actual_count = ifs.gcount();

  if (actual_count != scount)
  {
    std::stringstream msg;
    msg << "Requested " << scount << " from stream but " << actual_count << " were read";
    throw rosbaz::IoException(msg.str());
  }
}

}  // namespace io

}  // namespace rosbaz