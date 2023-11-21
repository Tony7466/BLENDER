/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup GHOST
 */

#pragma once

#ifndef WITH_INPUT_GAMEPAD
#  error Gamepad code included in non-gamepad-enabled build
#endif

#include "GHOST_Event.hh"

/**
 * Gamepad trigger event.
 * Events that contains input reading from gamepad triggers. Gamepad triggers generates analog data
 * and this input data can change so often. These events are only sent one in a frame and only
 * send the data from the active trigger snapshots.
 */
class GHOST_EventGamepadTrigger : public GHOST_Event {
 protected:
  GHOST_TEventGamepadTriggerData _trigger_data;

 public:
  GHOST_EventGamepadTrigger(uint64_t time, GHOST_IWindow *window)
      : GHOST_Event(time, GHOST_kEventGamepadTrigger, window)
  {
    m_data = &_trigger_data;
  }
};

/**
 * Gamepad thumbstick event.
 * Events that contains input reading from gamepad thumbsticks. Gamepad thumbsticks generates
 * analog data and this input data can change so often. These events are only sent one in a frame
 * and only send the data from the active thumbsticks snapshots.
 */
class GHOST_EventGamepadThumb : public GHOST_Event {
 protected:
  GHOST_TEventGamepadThumbData _thumb_data;

 public:
  GHOST_EventGamepadThumb(uint64_t time, GHOST_IWindow *window)
      : GHOST_Event(time, GHOST_kEventGamepadThumb, window)
  {
    m_data = &_thumb_data;
  }
};

/**
 * Gamepad button event.
 */
class GHOST_EventGamepadButton : public GHOST_Event {
 protected:
  GHOST_TEventGamepadButtonData _button_data;

 public:
  GHOST_EventGamepadButton(uint64_t time, GHOST_IWindow *window)
      : GHOST_Event(time, GHOST_kEventGamepadButton, window)
  {
    m_data = &_button_data;
  }
};
