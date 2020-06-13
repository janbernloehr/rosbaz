#include "pausable_context.h"

#include <chrono>
#include <csignal>
#include <cstdint>
#include <termios.h>
#include <thread>

#include <ros/console.h>
#include <ros/ros.h>

namespace
{
termios g_original_termios_flags;

void resetTerminal(int signal = 0)
{
  const int fd = fileno(stdin);
  tcsetattr(fd, TCSANOW, &g_original_termios_flags);
  if (ros::ok())
  {
    ros::shutdown();
  }
  if (signal != 0)
  {
    ROS_WARN_STREAM("Process stopped with signal " << signal);
    std::exit(-signal);
  }
}
}  // namespace

namespace terminal
{
PausableContext::PausableContext(bool is_paused) : is_paused_(is_paused)
{
  hideUserInputOnTerminal();
}

PausableContext::~PausableContext()
{
  resetTerminal();
}

bool PausableContext::tick()
{
  readKeyboardInput();

  if (!is_paused_)
  {
    return true;
  }

  if (steps_to_perform_ > 0)
  {
    --steps_to_perform_;
    return true;
  }

  return false;
}

char PausableContext::readOneChar() const
{
  fd_set testfd = stdin_fdset_;
  timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 0;
  if (select(maxfd_, &testfd, nullptr, nullptr, &tv) <= 0)
    return EOF;

  return static_cast<char>(getc(stdin));
}

void PausableContext::hideUserInputOnTerminal()
{
  const int fd = fileno(stdin);
  termios flags;
  tcgetattr(fd, &flags);
  g_original_termios_flags = flags;
  flags.c_lflag &= ~(ICANON | ECHO);
  flags.c_cc[VMIN] = 0;
  flags.c_cc[VTIME] = 0;
  tcsetattr(fd, TCSANOW, &flags);

  FD_ZERO(&stdin_fdset_);
  FD_SET(fd, &stdin_fdset_);
  maxfd_ = fd + 1;

  std::signal(SIGINT, resetTerminal);
  std::signal(SIGTERM, resetTerminal);
  std::signal(SIGKILL, resetTerminal);
  std::signal(SIGSEGV, resetTerminal);
}

void PausableContext::readKeyboardInput()
{
  std::int8_t ch;

  do
  {
    ch = readOneChar();

    switch (ch)
    {
      case kSingleStepKey:
        ++steps_to_perform_;
        break;
      case kPauseToggleKey:
        is_paused_ = !is_paused_;
        break;
    }
  } while (ch != EOF);
}
}  // namespace terminal
