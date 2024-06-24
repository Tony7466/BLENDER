/* SPDX-FileCopyrightText: 2002-2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#ifndef WITH_INPUT_NDOF
#  error NDOF code included in non-NDOF-enabled build
#endif

#include "GHOST_System.hh"
#include <array>

typedef enum {
  NDOF_UnknownDevice = 0,

  /* Current devices. */
  NDOF_SpaceNavigator,
  NDOF_SpaceExplorer,
  NDOF_SpacePilotPro,
  NDOF_SpaceMousePro,
  NDOF_SpaceMouseWireless,
  NDOF_SpaceMouseProWireless,
  NDOF_SpaceMouseEnterprise,
  NDOF_KeyboardPro,
  NDOF_NumpadPro,

  /* Older devices. */
  NDOF_SpacePilot,
  NDOF_Spaceball5000,
  NDOF_SpaceTraveler

} NDOF_DeviceT;

/**
 * NDOF device button event types.
 *
 * \note Button values are stored in DNA as part of key-map items.
 * Existing values should not be changed. Otherwise, a mapping must be used,
 * see #NDOF_BUTTON_INDEX_AS_EVENT.
 */
typedef enum {

  NDOF_BUTTON_NONE = -1,
  /* Used internally, never sent or used as an index. */
  NDOF_BUTTON_INVALID = 0,

  /* These two are available from any 3Dconnexion device. */
  NDOF_BUTTON_MENU = 1,
  NDOF_BUTTON_FIT = 2,

  /* Standard views. */
  NDOF_BUTTON_TOP = 3,
  NDOF_BUTTON_LEFT = 4,
  NDOF_BUTTON_RIGHT = 5,
  NDOF_BUTTON_FRONT = 6,
  NDOF_BUTTON_BOTTOM = 7,
  NDOF_BUTTON_BACK = 8,

  /* 90 degrees rotations. */
  NDOF_BUTTON_ROLL_CW = 9,
  NDOF_BUTTON_ROLL_CCW = 10,

  /* More views. */
  NDOF_BUTTON_ISO1 = 11,
  NDOF_BUTTON_ISO2 = 12,

  /* General-purpose buttons.
   * Users can assign functions via keymap editor. */
  NDOF_BUTTON_1 = 13,
  NDOF_BUTTON_2 = 14,
  NDOF_BUTTON_3 = 15,
  NDOF_BUTTON_4 = 16,
  NDOF_BUTTON_5 = 17,
  NDOF_BUTTON_6 = 18,
  NDOF_BUTTON_7 = 19,
  NDOF_BUTTON_8 = 20,
  NDOF_BUTTON_9 = 21,
  NDOF_BUTTON_10 = 22,

  /* Keyboard keys. */
  NDOF_BUTTON_ESC = 23,
  NDOF_BUTTON_ALT = 24,
  NDOF_BUTTON_SHIFT = 25,
  NDOF_BUTTON_CTRL = 26,

  /* Device control. */
  NDOF_BUTTON_ROTATE = 27,
  NDOF_BUTTON_PANZOOM = 28,
  NDOF_BUTTON_DOMINANT = 29,
  NDOF_BUTTON_PLUS = 30,
  NDOF_BUTTON_MINUS = 31,

  /* New spin buttons. */
  NDOF_BUTTON_SPIN_CW = 32,
  NDOF_BUTTON_SPIN_CCW = 33,
  NDOF_BUTTON_TILT_CW = 34,
  NDOF_BUTTON_TILT_CCW = 35,

  /* Keyboard keys. */
  NDOF_BUTTON_ENTER = 36,
  NDOF_BUTTON_DELETE = 37,

  /* Keyboard Pro special buttons. */
  NDOF_BUTTON_KBP_F1 = 41,
  NDOF_BUTTON_KBP_F2 = 42,
  NDOF_BUTTON_KBP_F3 = 43,
  NDOF_BUTTON_KBP_F4 = 44,
  NDOF_BUTTON_KBP_F5 = 45,
  NDOF_BUTTON_KBP_F6 = 46,
  NDOF_BUTTON_KBP_F7 = 47,
  NDOF_BUTTON_KBP_F8 = 48,
  NDOF_BUTTON_KBP_F9 = 49,
  NDOF_BUTTON_KBP_F10 = 50,
  NDOF_BUTTON_KBP_F11 = 51,
  NDOF_BUTTON_KBP_F12 = 52,

  /* General-purpose buttons.
   * Users can assign functions via keymap editor. */
  NDOF_BUTTON_11 = 77,
  NDOF_BUTTON_12 = 78,

  /* Store views. */
  NDOF_BUTTON_VIEW_1 = 103,
  NDOF_BUTTON_VIEW_2 = 104,
  NDOF_BUTTON_VIEW_3 = 105,
  NDOF_BUTTON_SAVE_VIEW_1 = 139,
  NDOF_BUTTON_SAVE_VIEW_2 = 140,
  NDOF_BUTTON_SAVE_VIEW_3 = 141,

  /* Keyboard keys. */
  NDOF_BUTTON_TAB = 175,
  NDOF_BUTTON_SPACE = 176,

  /* Numpad Pro special buttons. */
  NDOF_BUTTON_NP_F1 = 229,
  NDOF_BUTTON_NP_F2 = 230,
  NDOF_BUTTON_NP_F3 = 231,
  NDOF_BUTTON_NP_F4 = 232,

  /* add more here as needed - don't change value of anything that may already be used */
  NDOF_BUTTON_USER = 0x10000

} NDOF_ButtonT;

typedef std::array<NDOF_ButtonT, 6> NDOF_Button_Array;

typedef enum { ShortButton, LongButton } NDOF_Button_Type;

class GHOST_NDOFManager {
 public:
  GHOST_NDOFManager(GHOST_System &);
  virtual ~GHOST_NDOFManager() {}

  /**
   * Whether multi-axis functionality is available (via the OS or driver)
   * does not imply that a device is plugged in or being used.
   */
  virtual bool available() = 0;

  /**
   * Each platform's device detection should call this
   * use standard USB/HID identifiers.
   */
  bool setDevice(unsigned short vendor_id, unsigned short product_id);

  /**
   * Filter out small/accidental/un-calibrated motions by
   * setting up a "dead zone" around home position
   * set to 0 to disable
   * 0.1 is a safe and reasonable value.
   */
  void setDeadZone(float);

  /**
   * The latest raw axis data from the device.
   *
   * \note axis data should be in blender view coordinates
   * - +X is to the right.
   * - +Y is up.
   * - +Z is out of the screen.
   * - for rotations, look from origin to each +axis.
   * - rotations are + when CCW, - when CW.
   * Each platform is responsible for getting axis data into this form
   * these values should not be scaled (just shuffled or flipped).
   */
  void updateTranslation(const int t[3], uint64_t time);
  void updateRotation(const int r[3], uint64_t time);

  /**
   * The latest raw button data from the device
   * use HID button encoding (not #NDOF_ButtonT).
   */
  void updateButton(int button_number, bool press, uint64_t time);
  void updateButtonsBitmask(int button_bits, uint64_t time);
  void updateButtonsArray(NDOF_Button_Array buttons, uint64_t time, NDOF_Button_Type type);
  /* #NDOFButton events are sent immediately */

  /**
   * Processes and sends most recent raw data as an #NDOFMotion event
   * returns whether an event was sent.
   */
  bool sendMotionEvent();

 protected:
  GHOST_System &system_;

 private:
  void sendButtonEvent(NDOF_ButtonT, bool press, uint64_t time, GHOST_IWindow *);
  void sendKeyEvent(GHOST_TKey, bool press, uint64_t time, GHOST_IWindow *);

  NDOF_DeviceT device_type_;
  int hid_map_button_num_;
  int hid_map_button_mask_;
  const NDOF_ButtonT *hid_map_;

  int translation_[3];
  int rotation_[3];

  int button_depressed_;
  NDOF_Button_Array pressed_buttons_cache_;
  NDOF_Button_Array pressed_long_buttons_cache_;

  uint64_t motion_time_;      /* In milliseconds. */
  uint64_t motion_time_prev_; /* Time of most recent motion event sent. */

  GHOST_TProgress motion_state_;
  bool motion_event_pending_;
  float motion_dead_zone_; /* Discard motion with each component < this. */

  inline static std::array<NDOF_DeviceT, 9> bitmask_devices_ = {
      NDOF_SpaceNavigator,
      NDOF_SpaceExplorer,
      NDOF_SpacePilotPro,
      NDOF_SpaceMousePro,
      NDOF_SpaceMouseWireless,
      NDOF_SpaceMouseProWireless,
      NDOF_SpacePilot,
      NDOF_Spaceball5000,
      NDOF_SpaceTraveler,
  };
};
