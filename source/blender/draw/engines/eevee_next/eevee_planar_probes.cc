/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "eevee_planar_probes.hh"
#include "eevee_instance.hh"

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
  int2 render_extent = instance_.film.render_extent_get();
  render_extent.x = max_ii(render_extent.x >> light_probe->resolution_scale, 1);
  render_extent.y = max_ii(render_extent.y >> light_probe->resolution_scale, 1);
  probe.extent = render_extent;
  probe.object_mat = float4x4(ob->object_to_world);
  probe.clipping_offset = light_probe->clipsta;
}

void PlanarProbeModule::end_sync()
{
  remove_unused_probes();
  update_resources();
}

void PlanarProbeModule::update_resources()
{
  const int64_t num_probes = probes_.size();
  if (resources_.size() != num_probes) {
    resources_.reinitialize(num_probes);
  }

  int64_t resource_index = 0;
  for (PlanarProbe &probe : probes_.values()) {
    probe.resource_index = resource_index++;
    PlanarProbeResources &resources = resources_[probe.resource_index];
    resources.color_tx.ensure_2d(GPU_R11F_G11F_B10F,
                                 probe.extent,
                                 GPU_TEXTURE_USAGE_ATTACHMENT | GPU_TEXTURE_USAGE_SHADER_READ);
    resources.depth_tx.ensure_2d(GPU_DEPTH_COMPONENT32F,
                                 probe.extent,
                                 GPU_TEXTURE_USAGE_ATTACHMENT | GPU_TEXTURE_USAGE_SHADER_READ);
  }
}

PlanarProbe &PlanarProbeModule::find_or_insert(ObjectHandle &ob_handle)
{
  PlanarProbe &planar_probe = probes_.lookup_or_add_default(ob_handle.object_key.hash());
  return planar_probe;
}

void PlanarProbeModule::remove_unused_probes()
{
  probes_.remove_if(
      [](const PlanarProbes::MutableItem &item) { return !item.value.is_probe_used; });
}

PlanarProbeResources &PlanarProbeModule::resources_get(const PlanarProbe &probe)
{
  return resources_[probe.resource_index];
}

/** \} */

}  // namespace blender::eevee