#include "progress_bar.h"

#include <cmath>
#include <iostream>
#include <sstream>
#include <sys/ioctl.h>
#include <unistd.h>

namespace {
int digits(const int i) {
  return i > 0 ? static_cast<int>(std::log10(i)) + 1 : 1;
}
} // namespace

namespace terminal {

ProgressBar::ProgressBar(int max_steps) : ProgressBar(max_steps, 0, -1) {}
ProgressBar::ProgressBar(int max_steps, int start_step, int stop_step)
    : max_steps_(max_steps),
      start_step_(start_step < 0 ? max_steps + start_step : start_step),
      stop_step_(stop_step < 0 ? max_steps + 1 + stop_step : stop_step) {}

ProgressBar &ProgressBar::operator++() {
  ++current_step_;
  return *this;
}
ProgressBar &ProgressBar::operator+=(int value) {
  current_step_ += value;
  return *this;
}

void ProgressBar::display(const std::string &prefix) const {
  struct winsize w;
  int window_width;
  if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &w) != 0) {
    window_width = 80;
  } else {
    window_width = w.ws_col;
  }

  std::stringstream cmd_line;

  cmd_line << prefix;

  // Fill with whitespace so that <iteration> / <iterations> has constant width
  const int padding = digits(max_steps_) - digits(current_step_);
  for (int i = 0; i < padding; ++i) {
    cmd_line << ' ';
  }
  cmd_line << current_step_ << " / " << max_steps_;

  cmd_line.seekg(0, std::ios::end);

  const int remaining_space = static_cast<int>(window_width - cmd_line.tellg());
  const int bar_width = remaining_space - 3;
  const int bar_width_fill = (bar_width * current_step_) / max_steps_;

  int bar_idx_left_border;
  if (start_step_ == 0) {
    // do not show left marker
    bar_idx_left_border = -1;
  } else {
    bar_idx_left_border = (bar_width * start_step_) / max_steps_;
  }

  int bar_idx_right_border;
  if (stop_step_ == max_steps_) {
    // do not show right marker
    bar_idx_right_border = bar_width + 1;
  } else {
    bar_idx_right_border = (bar_width * stop_step_) / max_steps_;
  }

  cmd_line << "▐";
  for (int i = 0; i < bar_width; ++i) {
    if (i < bar_idx_left_border) {
      cmd_line << "░";
    } else if (i == bar_idx_left_border) {
      cmd_line << "╣";
    } else if (i < bar_idx_right_border) {
      if (i <= bar_width_fill) {
        cmd_line << "▒";
      } else {
        cmd_line << " ";
      }
    } else if (i == bar_idx_right_border) {
      cmd_line << "╠";
    } else {
      cmd_line << "░";
    }
  }
  cmd_line << "▌";

  std::cout << cmd_line.str() << '\r';
  std::cout << std::flush;
}
} // namespace terminal
