#include "rosbaz/io/io_helpers.h"

#include <array>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "rosbaz/common.h"

namespace rosbaz {

namespace io {

size_t size_of_file(std::string file_path) {
  RosStream file(file_path, std::ios_base::binary | std::ios_base::ate);
  return file.tellg();
}

void read_from(RosStream& ifs, byte* target, const size_t count) {
  ifs.read(reinterpret_cast<char*>(target), count);

  const auto actual_count = ifs.gcount();

  if (actual_count != count) {
    std::stringstream msg;
    msg << "Requested " << count << " from stream but read " << actual_count;
    throw std::runtime_error(msg.str());
  }
}

void read_from(RosStream& ifs, std::vector<byte>& target, const size_t count) {
  target.resize(count);
  byte* begin = &(*target.begin());
  read_from(ifs, begin, count);
}

}  // namespace io

}  // namespace rosbaz