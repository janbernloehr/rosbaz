#pragma once

#include <stdexcept>

namespace rosbaz
{
class Exception : public std::runtime_error
{
public:
  explicit Exception(const std::string& what) : std::runtime_error(what)
  {
  }
};

class InvalidUrlException : public Exception
{
public:
  explicit InvalidUrlException(std::string const& msg) : Exception(msg)
  {
  }
};

class MissingCredentialsException : public Exception
{
public:
  explicit MissingCredentialsException(std::string const& msg) : Exception(msg)
  {
  }
};

class IoException : public Exception
{
public:
  explicit IoException(std::string const& msg) : Exception(msg)
  {
  }
};

class UnsupportedRosBagException : public Exception
{
public:
  explicit UnsupportedRosBagException(std::string const& msg) : Exception(msg)
  {
  }
};

class RosBagFormatException : public Exception
{
public:
  explicit RosBagFormatException(std::string const& msg) : Exception(msg)
  {
  }
};

class InvalidBagIndexException : public Exception
{
public:
  explicit InvalidBagIndexException(std::string const& msg) : Exception(msg)
  {
  }
};

}  // namespace rosbaz