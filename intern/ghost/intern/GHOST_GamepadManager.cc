/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "GHOST_GamepadManager.hh"
#include "GHOST_EventGamepad.hh"
#include "GHOST_WindowManager.hh"
#include <ratio>

/** Active gamepad triggers snapshots. */
struct GHOST_ThumbState {
  float values[2];
  GHOST_TButtonAction state;
  const GHOST_TGamepadThumb type;
};

/** Active gamepad triggers snapshots. */
struct GHOST_TriggerState {
  float value;
  GHOST_TButtonAction state;
  const GHOST_TGamepadTrigger type;
};

/** Active gamepad input snapshot. */
struct GHOST_GamepadState {

  GHOST_ThumbState left_thumb{{0.0f}, GHOST_kRelease, GHOST_kGamepadLeftThumb};
  GHOST_ThumbState right_thumb{{0.0f}, GHOST_kRelease, GHOST_kGamepadRightThumb};

  GHOST_TriggerState left_trigger{{0.0f}, GHOST_kRelease, GHOST_kGamepadLeftTrigger};
  GHOST_TriggerState right_trigger{{0.0f}, GHOST_kRelease, GHOST_kGamepadRightTrigger};

  std::bitset<14> button_depressed{false};
  float dead_zone{0.0f};
};

static const char *gamepad_button_names[] = {
    "A",
    "B",
    "X",
    "Y",
    "LEFT_SHOULDER",
    "RIGHT_SHOULDER",
    "VIEW",
    "MENU",
    "LEFT_THUMB",
    "RIGHT_THUMB",
    "DPAD_UP",
    "DPAD_RIGHT",
    "DPAD_DOWN",
    "DPAD_LEFT",
};

GHOST_GamepadManager::GHOST_GamepadManager(GHOST_System &sys) : _system(sys), _dead_zone{0.3}
{
  _gamepad_state = std::make_unique<GHOST_GamepadState>();
}

GHOST_GamepadManager::~GHOST_GamepadManager() {}

void GHOST_GamepadManager::send_button_event(GHOST_TGamepadButton button,
                                             bool press,
                                             uint64_t time,
                                             GHOST_IWindow *window)
{
  GHOST_EventGamepadButton *event = new GHOST_EventGamepadButton(time, window);
  GHOST_TEventGamepadButtonData *data = (GHOST_TEventGamepadButtonData *)event->getData();

  data->action = press ? GHOST_kPress : GHOST_kRelease;
  data->button = button;

  _system.pushEvent(event);
}

void GHOST_GamepadManager::update_gamepad_state(const bool button_state[14], const float axis[6])
{

  _gamepad_state->left_thumb.values[0] = axis[0];
  _gamepad_state->left_thumb.values[1] = axis[1];

  _gamepad_state->right_thumb.values[0] = axis[2];
  _gamepad_state->right_thumb.values[1] = axis[3];

  _gamepad_state->left_trigger.value = axis[4];
  _gamepad_state->right_trigger.value = axis[5];

  GHOST_IWindow *window = _system.getWindowManager()->getActiveWindow();

  const uint64_t now = _system.getMilliSeconds();

  for (int x = 0; x < 14; x++) {
    bool button_depressed = _gamepad_state->button_depressed.test(x);
    if (button_depressed != button_state[x]) {
      /** Send buttons events only if the button status changed. */
      send_button_event(GHOST_TGamepadButton(x), button_state[x], now, window);
      _gamepad_state->button_depressed.set(x, button_state[x]);
    }
  }
}

void GHOST_GamepadManager::set_dead_zone(const float dz)
{
  _dead_zone = std::max(dz, 0.0f);
}

bool GHOST_GamepadManager::send_gamepad_frame_events(const float delta_time)
{
  GHOST_IWindow *window = _system.getWindowManager()->getActiveWindow();

  const uint64_t now = _system.getMilliSeconds();

  if (window == nullptr) {
    return false;
  }
  if (!_gamepad_state) {
    return false;
  }
  auto test_dead_zone = [this](float value) {
    return std::abs(value) < _dead_zone ? 0.0f : value;
  };

  int pushed_events = 0;

  auto send_thumb_event = [&, this](GHOST_ThumbState &thumb) {
    const float _values[]{test_dead_zone(thumb.values[0]), test_dead_zone(thumb.values[1])};
    const bool in_depressed = (_values[0] || _values[1]);

    /** Send only thumbstick events if there is non-zero reading or the thumbstick are released. */
    if (!in_depressed && thumb.state == GHOST_kRelease) {
      return 0;
    }

    GHOST_EventGamepadThumb *event = new GHOST_EventGamepadThumb(now, window);
    GHOST_TEventGamepadThumbData *data = (GHOST_TEventGamepadThumbData *)event->getData();
    data->value[0] = _values[0];
    data->value[1] = _values[1];
    data->thumb = thumb.type;
    data->action = in_depressed ? GHOST_kPress : GHOST_kRelease;
    data->dt = delta_time;
    thumb.state = data->action;
    _system.pushEvent(event);
    return 1;
  };

  pushed_events += send_thumb_event(_gamepad_state->left_thumb);
  pushed_events += send_thumb_event(_gamepad_state->right_thumb);

  auto send_trigger_event = [&, this](GHOST_TriggerState &trigger) {
    const float _value{test_dead_zone(trigger.value)};
    const bool in_depressed = (_value);

    if (!in_depressed && trigger.state == GHOST_kRelease) {
      return 0;
    }

    /** Send only triggers events if there is non-zero reading or the triggers are released. */
    GHOST_EventGamepadTrigger *event = new GHOST_EventGamepadTrigger(now, window);
    GHOST_TEventGamepadTriggerData *data = (GHOST_TEventGamepadTriggerData *)event->getData();
    data->value = _value;
    data->trigger = trigger.type;
    data->action = in_depressed ? GHOST_kPress : GHOST_kRelease;
    data->dt = delta_time;
    trigger.state = data->action;
    _system.pushEvent(event);
    return 1;
  };

  pushed_events += send_trigger_event(_gamepad_state->left_trigger);
  pushed_events += send_trigger_event(_gamepad_state->right_trigger);

  for (int x = 0; x < 14; x++) {
    bool button_depressed = _gamepad_state->button_depressed.test(x);
    if (button_depressed) {
      send_button_event(GHOST_TGamepadButton(x), button_depressed, now, window);
    }
  }

  return pushed_events > 0;
}

bool GHOST_GamepadManager::reset_gamepad_state()
{
  _gamepad_state->left_thumb.values[0] = 0.0f;
  _gamepad_state->left_thumb.values[1] = 0.0f;

  _gamepad_state->right_thumb.values[0] = 0.0f;
  _gamepad_state->right_thumb.values[1] = 0.0f;

  _gamepad_state->left_trigger.value = 0.0f;
  _gamepad_state->right_trigger.value = 0.0f;

  for (int x = 0; x < 14; x++) {
    _gamepad_state->button_depressed[x] = false;
  }
  return true;
}
