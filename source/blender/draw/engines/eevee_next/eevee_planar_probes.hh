/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup eevee
 */

#pragma once

#include "eevee_shader_shared.hh"

#include "BKE_cryptomatte.hh"

extern "C" {
struct Material;
}

namespace blender::eevee {

class Instance;
struct ObjectHandle;

/* -------------------------------------------------------------------- */
/** \name Planar Probe
 * \{ */

struct PlanarProbe : NonCopyable {
  Texture probes_tx = {"Planar"};
  float4x4 object_mat;
  float2 clipping_distances;
  int resolution;

  bool do_render = false;
  bool is_probe_used = false;
};

/** \} */

/* -------------------------------------------------------------------- */
/** \name Planar Probe Module
 * \{ */

class PlanarProbeModule {
 private:
  /* Max resolution of a texture. */
  static constexpr int max_resolution_ = 2048;

  Instance &instance_;
  Map<uint64_t, PlanarProbe> probes_;

  bool update_probes_next_sample_ = false;
  bool update_probes_this_sample_ = false;

 public:
  PlanarProbeModule(Instance &instance) : instance_(instance) {}

  void init();
  void begin_sync();
  void sync_object(Object *ob, ObjectHandle &ob_handle);
  void end_sync();

  template<typename T> void bind_resources(draw::detail::PassBase<T> *pass) {}

  const std::optional<std::reference_wrapper<PlanarProbe>> update_pop();

 private:
  PlanarProbe &find_or_insert(ObjectHandle &ob_handle);
  void remove_unused_probes();

  friend class Instance;
};

/** \} */

}  // namespace blender::eevee
