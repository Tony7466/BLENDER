/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_bit_vector.hh"

#include "eevee_instance.hh"
#include "eevee_planar_probes.hh"

#include <iostream>

namespace blender::eevee {

/* -------------------------------------------------------------------- */
/** \name Planar Probe Module
 * \{ */

void PlanarProbeModule::init()
{
  update_probes_ = !probes_.is_empty();
}

void PlanarProbeModule::begin_sync()
{
  for (PlanarProbe &planar_probe : probes_.values()) {
    planar_probe.is_probe_used = false;
  }
}

void PlanarProbeModule::sync_object(Object *ob, ObjectHandle &ob_handle)
{
  const ::LightProbe *light_probe = (::LightProbe *)ob->data;
  if (light_probe->type != LIGHTPROBE_TYPE_PLANAR) {
    return;
  }

  if (probes_.is_empty()) {
    update_probes_ = true;
    instance_.sampling.reset();
  }

  PlanarProbe &probe = find_or_insert(ob_handle);
  probe.is_probe_used = true;
  probe.resolution = 1 << light_probe->resolution;
  probe.object_mat = float4x4(ob->object_to_world);
  probe.clipping_distance = light_probe->clipsta;
}

void PlanarProbeModule::end_sync()
{
  remove_unused_probes();
}

PlanarProbe &PlanarProbeModule::find_or_insert(ObjectHandle &ob_handle)
{
  PlanarProbe &planar_probe = probes_.lookup_or_add_cb(ob_handle.object_key.hash(), [this]() {
    PlanarProbe probe;
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

/** \} */

}  // namespace blender::eevee