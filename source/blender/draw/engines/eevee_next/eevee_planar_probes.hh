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

struct PlanarProbe {
  float4x4 object_mat;
  float clipping_offset;
  int2 extent;
  bool is_probe_used = false;
  int resource_index;
};

struct PlanarProbeResources : NonCopyable {
  Texture color_tx = {"PlanarColor"};
  Texture depth_tx = {"PlanarDepth"};
  Framebuffer framebuffer = {"Planar"};
};

/** \} */

/* -------------------------------------------------------------------- */
/** \name Planar Probe Module
 * \{ */

class PlanarProbeModule {
  using PlanarProbes = Map<uint64_t, PlanarProbe>;
  using Resources = Array<PlanarProbeResources>;

 private:
  /* Max resolution of a texture. */
  static constexpr int max_resolution_ = 2048;

  Instance &instance_;
  PlanarProbes probes_;
  Resources resources_;

  bool update_probes_ = false;

 public:
  PlanarProbeModule(Instance &instance) : instance_(instance) {}

  void init();
  void begin_sync();
  void sync_object(Object *ob, ObjectHandle &ob_handle);
  void end_sync();

  template<typename T> void bind_resources(draw::detail::PassBase<T> *pass) {}

  PlanarProbeResources &resources_get(const PlanarProbe &probe);

 private:
  PlanarProbe &find_or_insert(ObjectHandle &ob_handle);
  void remove_unused_probes();
  void update_resources();

  friend class Instance;
  friend class CapturePlanarView;
};

/** \} */

}  // namespace blender::eevee
