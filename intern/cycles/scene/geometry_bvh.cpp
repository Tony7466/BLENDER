/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include "bvh/bvh.h"
#include "bvh/bvh2.h"

#include "device/device.h"

#include "scene/attribute.h"
#include "scene/camera.h"
#include "scene/geometry.h"
#include "scene/hair.h"
#include "scene/light.h"
#include "scene/mesh.h"
#include "scene/object.h"
#include "scene/pointcloud.h"
#include "scene/scene.h"
#include "scene/shader.h"
#include "scene/shader_nodes.h"
#include "scene/stats.h"
#include "scene/volume.h"

#include "subd/patch_table.h"
#include "subd/split.h"

#include "kernel/osl/globals.h"

#include "util/foreach.h"
#include "util/log.h"
#include "util/progress.h"
#include "util/task.h"

CCL_NAMESPACE_BEGIN

void Geometry::compute_bvh(Device *device,
                           DeviceScene *dscene,
                           SceneParams *params,
                           Progress *progress,
                           size_t n,
                           size_t total)
{
  if (progress->get_cancel())
    return;

  const BVHLayout bvh_layout = BVHParams::best_bvh_layout(
      params->bvh_layout, device->get_bvh_layout_mask(dscene->data.kernel_features));
  if (need_build_bvh(bvh_layout)) {
    BVH *sub_bvh = bvh->get_device_bvh(device);
    GeometryManager::device_update_sub_bvh(
        device, dscene, bvh, sub_bvh, !need_update_rebuild, n, total, progress);
  }
}

void GeometryManager::device_init_update_bvh(Scene *scene)
{
  if (scene->bvh->params.bvh_layout == BVH_LAYOUT_BVH2) {
    /* To ensure that only 1 BVH2 scene is built a count of workers is used */
    BVH2 *const bvh2 = static_cast<BVH2 *>(scene->bvh);
    bvh2->building++;
  }
}

void GeometryManager::device_update_bvh(Device *device,
                                        DeviceScene *dscene,
                                        Scene *scene,
                                        bool can_refit,
                                        size_t n,
                                        size_t total,
                                        Progress &progress)
{
  BVH *bvh = scene->bvh;
  BVH *sub_bvh = scene->bvh->get_device_bvh(device);
  GeometryManager::device_update_sub_bvh(
      device, dscene, bvh, sub_bvh, can_refit, n, total, &progress);
}

void GeometryManager::device_update_bvh_postprocess(Device *device,
                                                    DeviceScene *dscene,
                                                    Scene *scene,
                                                    Progress &progress)
{
  BVH *bvh = scene->bvh;

  const bool has_bvh2_layout = (bvh->params.bvh_layout == BVH_LAYOUT_BVH2);

  if (has_bvh2_layout) {
    BVH2 *bvh2 = static_cast<BVH2 *>(scene->bvh);
    PackedBVH pack = std::move(bvh2->pack);
    dscene->data.bvh.root = pack.root_index;
  }
  else {
    dscene->data.bvh.root = -1;
  }

  dscene->data.bvh.use_bvh_steps = (scene->params.num_bvh_time_steps != 0);
  dscene->data.bvh.curve_subdivisions = scene->params.curve_subdivisions();
  dscene->data.device_bvh = 0;
}

bool Geometry::create_new_bvh_if_needed(Object *object,
                                        Device *device,
                                        DeviceScene *dscene,
                                        SceneParams *params)
{
  bool status = false;
  const BVHLayout bvh_layout = BVHParams::best_bvh_layout(
      params->bvh_layout, device->get_bvh_layout_mask(dscene->data.kernel_features));
  if (need_build_bvh(bvh_layout)) {
    /* Ensure all visibility bits are set at the geometry level BVH. In
     * the object level BVH is where actual visibility is tested. */
    object->set_is_shadow_catcher(true);
    object->set_visibility(~0);

    object->set_geometry(this);

    vector<Geometry *> geometry;
    geometry.push_back(this);
    vector<Object *> objects;

    objects.push_back(object);

    if (bvh && !need_update_rebuild) {
      bvh->replace_geometry(geometry, objects);
    }
    else {
      if (bvh != NULL) {
        delete bvh;
      }

      BVHParams bparams;
      bparams.use_spatial_split = params->use_bvh_spatial_split;
      bparams.use_compact_structure = params->use_bvh_compact_structure;
      bparams.bvh_layout = bvh_layout;
      bparams.use_unaligned_nodes = dscene->data.bvh.have_curves &&
                                    params->use_bvh_unaligned_nodes;
      bparams.num_motion_triangle_steps = params->num_bvh_time_steps;
      bparams.num_motion_curve_steps = params->num_bvh_time_steps;
      bparams.num_motion_point_steps = params->num_bvh_time_steps;
      bparams.bvh_type = params->bvh_type;
      bparams.curve_subdivisions = params->curve_subdivisions();

      bvh = BVH::create(bparams, geometry, objects, device);
      need_update_rebuild = true;
    }
    status = true;
  }

  return status;
}

void GeometryManager::device_update_sub_bvh(Device *device,
                                            DeviceScene *dscene,
                                            BVH *bvh,
                                            BVH *sub_bvh,
                                            bool can_refit,
                                            size_t n,
                                            size_t total,
                                            Progress *progress)
{
  string msg = "Updating Geometry BVH";

  // Is this a multi-bvh?
  if (sub_bvh && can_refit) {
    progress->set_status(msg, "Refitting BVH");
    // Don't redo the setup if this is not a sub-bvh
    if (sub_bvh != bvh) {
      sub_bvh->replace_geometry(bvh->geometry, bvh->objects);
    }
  }
  else {
    progress->set_status(msg, "Building BVH");
    // Don't redo the setup if this is not a sub-bvh
    if (sub_bvh != bvh) {
      // Yes, so setup the device specific sub_bvh in the multi-bvh.
      BVHParams bparams = bvh->params;
      // Set the layout to the correct one for the device
      bparams.bvh_layout = device->get_bvh_layout(device, bvh->params.bvh_layout);
      if (sub_bvh != NULL) {
        delete sub_bvh;
      }
      VLOG_INFO << "Sub-BVH using layout " << bvh_layout_name(bparams.bvh_layout) << " from layout " << bvh_layout_name(bvh->params.bvh_layout);
      /* BVH2 should not have a sub-bvh as only 1 is built on the CPU */
      assert(bparams.bvh_layout != BVH_LAYOUT_BVH2); 
      if(bparams.bvh_layout != BVH_LAYOUT_BVH2) {
	sub_bvh = BVH::create(bparams, bvh->geometry, bvh->objects, device);
	bvh->set_device_bvh(device, sub_bvh);
      }
    }
    can_refit = false;
  }
  device->build_bvh(sub_bvh, dscene, *progress, can_refit);
}

bool GeometryManager::device_update_bvh_preprocess(Device *device,
                                                   DeviceScene *dscene,
                                                   Scene *scene,
                                                   Progress &progress)
{
  /* bvh build */

  BVHParams bparams;
  bparams.top_level = true;
  bparams.bvh_layout = BVHParams::best_bvh_layout(
      scene->params.bvh_layout, device->get_bvh_layout_mask(dscene->data.kernel_features));
  bparams.use_spatial_split = scene->params.use_bvh_spatial_split;
  bparams.use_unaligned_nodes = dscene->data.bvh.have_curves &&
                                scene->params.use_bvh_unaligned_nodes;
  bparams.num_motion_triangle_steps = scene->params.num_bvh_time_steps;
  bparams.num_motion_curve_steps = scene->params.num_bvh_time_steps;
  bparams.num_motion_point_steps = scene->params.num_bvh_time_steps;
  bparams.bvh_type = scene->params.bvh_type;
  bparams.curve_subdivisions = scene->params.curve_subdivisions();

  VLOG_INFO << "Using " << bvh_layout_name(bparams.bvh_layout) << " layout.";

  const bool can_refit = scene->bvh != nullptr &&
                         (bparams.bvh_layout == BVHLayout::BVH_LAYOUT_OPTIX ||
                          bparams.bvh_layout == BVHLayout::BVH_LAYOUT_METAL ||
                          bparams.bvh_layout == BVHLayout::BVH_LAYOUT_MULTI_OPTIX ||
                          bparams.bvh_layout == BVHLayout::BVH_LAYOUT_MULTI_METAL);

  BVH *bvh = scene->bvh;
  if (!scene->bvh) {
    bvh = scene->bvh = BVH::create(bparams, scene->geometry, scene->objects, device);
  }

  /* Mark BVH as having not been built yet */
  bvh->built = false;
  return can_refit;
}

/*
 * Creates a new BVH for the geometry if it is needed otherwise
 * it determines if the BVH can be refitted. It also counts
 * the number of BVH that need to be built.
 */
size_t GeometryManager::create_object_bvhs(Device *device,
                                         DeviceScene *dscene,
                                         Scene *scene,
                                         const BVHLayout bvh_layout,
                                         bool &need_update_scene_bvh)
{
  scoped_callback_timer timer([scene](double time) {
    if (scene->update_stats) {
      scene->update_stats->geometry.times.add_entry(
          {"device_update (object BVHs preprocess)", time});
    }
  });
  size_t num_bvh = 0;

  if (scene->geometry.size() > object_pool.size()) {
    object_pool.resize(scene->geometry.size());
  }

  // Create BVH structures where needed
  int id = 0;
  foreach (Geometry *geom, scene->geometry) {
    if (geom->is_modified() || geom->need_update_bvh_for_offset) {
      need_update_scene_bvh = true;
      Object *object = &object_pool[id];
      if(geom->create_new_bvh_if_needed(object, device, dscene, &scene->params)) {
        num_bvh++;
      }
    }
    id++;
  }

  return num_bvh;
}

CCL_NAMESPACE_END
