/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once
#ifndef WITH_INPUT_GAMEPAD
#  error Gamepad code included in non-Gamepad-enabled build
#endif

#include <bitset>

#include "GHOST_System.hh"

struct GHOST_GamepadState;

class GHOST_GamepadManager {
 public:
  GHOST_GamepadManager(GHOST_System &);
  virtual ~GHOST_GamepadManager();

  virtual bool available() = 0;

  /** Send gamepad frame events. */
  bool send_gamepad_frame_events(const float delta_time);

  void set_dead_zone(const float);

 protected:
  /** Reset the current status of the gamepad, should be used if a gamepad is not longer available.
   */
  bool reset_gamepad_state();

  /** Update the current status of the gamepad, and sends events for buttons that status changed.
   */
  void update_gamepad_state(const bool buttons[14], const float axis[6]);

 private:
  /** Send button events. */
  void send_button_event(GHOST_TGamepadButton button,
                         bool press,
                         uint64_t time,
                         GHOST_IWindow *window);

  std::unique_ptr<GHOST_GamepadState> _gamepad_state;
  GHOST_System &_system;
  float _dead_zone;
};
