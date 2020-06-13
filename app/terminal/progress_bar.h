#pragma once

#include <string>

namespace terminal
{
/// Prints a progress bar on the terminal.
///
/// \detail
/// The size of the progress bar is inferred automatically from the maximum
/// number of steps, the current step and the width of the terminal.
///
/// The progress bar can be incremented using the ++ and += operators as well as
/// setCurrentStep.
class ProgressBar
{
public:
  ProgressBar(int max_steps);
  ProgressBar(int max_steps, int start_step, int stop_step);

  ProgressBar& operator++();
  ProgressBar& operator+=(int value);

  /// Print the current state of the progress bar to the terminal.
  void display(const std::string& prefix = "") const;

  int getCurrentStep() const
  {
    return current_step_;
  }
  void setCurrentStep(int value)
  {
    current_step_ = value;
  }
  int getMaxSteps() const
  {
    return max_steps_;
  }

  int getStartStep() const
  {
    return start_step_;
  }
  int getStopStep() const
  {
    return stop_step_;
  }

  /// Reset the progress bar to its initial value.
  void reset()
  {
    current_step_ = 0;
  }

private:
  int current_step_ = 0;
  const int max_steps_ = 0;
  const int start_step_ = 0;
  const int stop_step_ = 0;
};
}  // namespace terminal
