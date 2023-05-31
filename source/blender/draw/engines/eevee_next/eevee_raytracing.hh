/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup eevee
 *
 * Screen Space Ray Tracing
 *
 */

#pragma once

#include "eevee_shader_shared.hh"

namespace blender::eevee {

class Instance;

/* -------------------------------------------------------------------- */
/** \name RayTracing
 * \{ */

class RayTracing {
 private:
  class Instance &inst_;

  RayTracingDataBuf data_;

 public:
  RayTracing(Instance &inst) : inst_(inst){};
  ~RayTracing(){};

  void init();

  template<typename T> void bind_resources(draw::detail::PassBase<T> *pass)
  {
    pass->bind_ubo(RAYTRACE_BUF_SLOT, &data_);
  }
};

/** \} */

}  // namespace blender::eevee
