/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "GHOST_GamepadManager.hh"

class GHOST_GamepadManagerWin32 : public GHOST_GamepadManager {
 private:
 public:
  GHOST_GamepadManagerWin32(GHOST_System &);
  ~GHOST_GamepadManagerWin32();

  bool available();

  /** Retrieves the current state of the gamepad reported by the system. */
  bool update_gamepad();
};
