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
  ifs.read(reinterpret_cast<char*>(target), count);

  const auto actual_count = ifs.gcount();

  if (actual_count != count)
  {
    std::stringstream msg;
    msg << "Requested " << count << " from stream but " << actual_count << " were read";
    throw rosbaz::IoException(msg.str());
  }
}

}  // namespace io

}  // namespace rosbaz