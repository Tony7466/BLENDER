#pragma once

#include "renderdoc_app.h"

namespace renderdoc::api {
class Renderdoc {
 private:
  enum class State {
    /**
     * Initial state of the API indicating that the API hasn't checked if it can find renderdoc.
     */
    UNINITIALIZED,

    /**
     * API has looked for renderdoc, but couldn't find it. This indicates that renderdoc isn't
     * available on the platform, or wasn't registered correctly.
     */
    NOT_FOUND,

    /**
     * API has loaded the symbols of renderdoc.
     */
    LOADED,
  };
  State state_ = State::UNINITIALIZED;
  RENDERDOC_API_1_6_0 *renderdoc_api_ = nullptr;

 public:
  void start_frame_capture(RENDERDOC_DevicePointer device_handle,
                           RENDERDOC_WindowHandle window_handle);
  void end_frame_capture(RENDERDOC_DevicePointer device_handle,
                         RENDERDOC_WindowHandle window_handle);

 private:
  bool check_loaded();
  void load();
};
}  // namespace renderdoc::api