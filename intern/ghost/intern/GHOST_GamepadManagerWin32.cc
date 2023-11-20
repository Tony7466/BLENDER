/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "GHOST_GamepadManagerWin32.hh"

#include <limits>

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#pragma comment(lib, "xinput")
#include <xinput.h>

GHOST_GamepadManagerWin32::GHOST_GamepadManagerWin32(GHOST_System &sys) : GHOST_GamepadManager(sys)
{
}

GHOST_GamepadManagerWin32::~GHOST_GamepadManagerWin32() {}

bool GHOST_GamepadManagerWin32::available()
{
  return true;
}

bool GHOST_GamepadManagerWin32::update_gamepad()
{
  XINPUT_STATE input_state{0};
  DWORD dwResult = XInputGetState(0, &input_state);
  
  if (dwResult == 0) {
    bool buttons[16];
    float axis_data[6];
    XINPUT_GAMEPAD &gamepad = input_state.Gamepad;

    constexpr float shrt_max_float = float(std::numeric_limits<short>::max());
    constexpr float uchar_max_float = float(std::numeric_limits<unsigned char>::max());
    
    axis_data[0] = float(gamepad.sThumbLX) / shrt_max_float;
    axis_data[1] = float(gamepad.sThumbLY) / shrt_max_float;
    axis_data[2] = float(gamepad.sThumbRX) / shrt_max_float;
    axis_data[3] = float(gamepad.sThumbRY) / shrt_max_float;
    axis_data[4] = float(gamepad.bLeftTrigger) / uchar_max_float;
    axis_data[5] = float(gamepad.bRightTrigger) / uchar_max_float;

    constexpr int button_mask[]{
        XINPUT_GAMEPAD_A,
        XINPUT_GAMEPAD_B,
        XINPUT_GAMEPAD_X,
        XINPUT_GAMEPAD_Y,

        XINPUT_GAMEPAD_LEFT_SHOULDER,
        XINPUT_GAMEPAD_RIGHT_SHOULDER,

        XINPUT_GAMEPAD_BACK,
        XINPUT_GAMEPAD_START,

        XINPUT_GAMEPAD_LEFT_THUMB,
        XINPUT_GAMEPAD_RIGHT_THUMB,

        XINPUT_GAMEPAD_DPAD_UP,
        XINPUT_GAMEPAD_DPAD_DOWN,
        XINPUT_GAMEPAD_DPAD_LEFT,
        XINPUT_GAMEPAD_DPAD_RIGHT,
    };

    for (int x = 0; x < 14; x++) {
      buttons[x] = bool(gamepad.wButtons & button_mask[x]);
    }

    update_gamepad_state(buttons, axis_data);
  }
  else {
    GHOST_GamepadManager::reset_gamepad_state();
  }
  return true;
}
