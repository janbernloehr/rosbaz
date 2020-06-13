#pragma once

#include <stdexcept>

namespace rosbaz
{
class Exception : public std::runtime_error
{
public:
  Exception(const std::string& what) : std::runtime_error(what)
  {
  }
};

class InvalidUrlException : public Exception
{
public:
  InvalidUrlException(std::string const& msg) : Exception(msg)
  {
  }
};

class MissingCredentialsException : public Exception
{
public:
  MissingCredentialsException(std::string const& msg) : Exception(msg)
  {
  }
};

class IoException : public Exception
{
public:
  IoException(std::string const& msg) : Exception(msg)
  {
  }
};

class UnsupportedRosBagException : public Exception
{
public:
  UnsupportedRosBagException(std::string const& msg) : Exception(msg)
  {
  }
};

class RosBagFormatException : public Exception
{
public:
  RosBagFormatException(std::string const& msg) : Exception(msg)
  {
  }
};

class InvalidBagIndexException : public Exception
{
public:
  InvalidBagIndexException(std::string const& msg) : Exception(msg)
  {
  }
};

}  // namespace rosbaz