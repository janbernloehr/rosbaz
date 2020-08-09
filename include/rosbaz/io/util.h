#pragma once

#include <type_traits>

namespace rosbaz
{
namespace io
{
template <class T, class U>
T narrow(U u) noexcept(false)
{
  T t = static_cast<T>(u);
  assert(static_cast<U>(t) == u);
  assert(std::is_signed<U>::value == std::is_signed<T>::value || ((t < T{}) == (u < U{})));
  return t;
}
}  // namespace io
}  // namespace rosbaz
