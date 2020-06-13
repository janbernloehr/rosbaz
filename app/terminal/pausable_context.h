#pragma once
#include <sys/select.h>

namespace terminal
{
/// Provides a user interface on the terminal to pause/unpause an operation
///
/// \detail
/// When instantiated, the PausableContext reconfigures the terminal so that
/// user input is not shown. This is reverted by the dtor as well as the SIGINT,
/// SIGTERM, and SIGABORT handlers.
///
/// The user can toggle between pause and unpause using the kPauseToggleKey.
/// When tick() is called, the user input is evaluated to determine the current
/// paused state. Moreover, in paused mode, the user can ask for a finite number
/// of steps to be executed using the kSingleStepKey. tick() will return true as
/// many times as this key was pressed.
class PausableContext
{
public:
  static constexpr char kPauseToggleKey = ' ';
  static constexpr char kSingleStepKey = 's';

  /// Creates the PausableContext in the given state and reconfigures the
  /// terminal to hide all user input.
  PausableContext(bool is_paused);

  /// Restores the terminal to allow user input.
  ~PausableContext();

  /// \return true if unpaused or if paused and there are steps left that the
  /// user requested with kSingleStepKey; in the latter case, the remaining
  /// steps will be decremented; otherwise false.
  bool tick();

private:
  void hideUserInputOnTerminal();
  char readOneChar() const;
  void readKeyboardInput();

  bool is_paused_ = true;
  int steps_to_perform_ = 0;

  // Terminal
  fd_set stdin_fdset_;
  int maxfd_ = -1;
};
}  // namespace terminal
