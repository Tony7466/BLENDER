/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_bit_vector.hh"

#include "eevee_instance.hh"
#include "eevee_planar_probes.hh"

#include <iostream>

namespace blender::eevee {

/* -------------------------------------------------------------------- */
/** \name Planar Probe
 * \{ */

/** \} */

/* -------------------------------------------------------------------- */
/** \name Planar Probe Module
 * \{ */

void PlanarProbeModule::init() {}
void PlanarProbeModule::begin_sync()
{
  for (PlanarProbe &planar_probe : probes_.values()) {
    planar_probe.is_probe_used = false;
  }

  update_probes_this_sample_ = false;
  if (update_probes_next_sample_) {
    update_probes_this_sample_ = true;
    instance_.sampling.reset();
  }
}

void PlanarProbeModule::sync_object(Object *ob, ObjectHandle &ob_handle)
{
  const ::LightProbe *light_probe = (::LightProbe *)ob->data;
  if (light_probe->type != LIGHTPROBE_TYPE_PLANAR) {
    return;
  }
  const bool is_dirty = ob_handle.recalc != 0;

  PlanarProbe &probe = find_or_insert(ob_handle);
  probe.do_render |= is_dirty;
  probe.is_probe_used = true;
  probe.resolution = 1 << light_probe->resolution;
  probe.object_mat = float4x4(ob->object_to_world);
  probe.clipping_distances = float2(light_probe->clipsta, light_probe->clipend);

  if (!instance_.do_planar_probe_sync()) {
    update_probes_next_sample_ = true;
    return;
  }
}

void PlanarProbeModule::end_sync()
{
  remove_unused_probes();
  const bool do_update = instance_.do_planar_probe_sync();
  if (!do_update) {
    return;
  }
}

PlanarProbe &PlanarProbeModule::find_or_insert(ObjectHandle &ob_handle)
{
  PlanarProbe &planar_probe = probes_.lookup_or_add_cb(ob_handle.object_key.hash(), [this]() {
    PlanarProbe probe;
    probe.do_render = true;
    return probe;
  });

  return planar_probe;
}

void PlanarProbeModule::remove_unused_probes()
{
  probes_.remove_if([](const Map<uint64_t, PlanarProbe>::MutableItem &item) {
    return !item.value.is_probe_used;
  });
}

const std::optional<std::reference_wrapper<PlanarProbe>> PlanarProbeModule::update_pop()
{
  const bool do_probe_sync = instance_.do_planar_probe_sync();
  if (!do_probe_sync) {
    return std::nullopt;
  }

  for (const Map<uint64_t, PlanarProbe>::MutableItem &item : probes_.items()) {
    PlanarProbe &probe = item.value;
    if (!probe.do_render) {
      continue;
    }

    probe.probes_tx.ensure_2d(GPU_RGBA32F, int2(probe.resolution));
    probe.do_render = false;

    return std::make_optional(std::reference_wrapper(probe));
  }

  /* Check reset probe updating as we completed rendering all Probes. */
  if (update_probes_this_sample_ && update_probes_next_sample_) {
    update_probes_next_sample_ = false;
  }

  return std::nullopt;
}

/** \} */

}  // namespace blender::eevee