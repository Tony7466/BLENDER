/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#ifndef WITH_INPUT_GAMEPAD
#  error Gamepad code included in non-gamepad-enabled build
#endif

#include "GHOST_Event.hh"

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
