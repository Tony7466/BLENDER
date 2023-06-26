/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include "bvh/bvh2.h"

#include "scene/devicescene.h"
#include "scene/scene.h"

#include "device/device.h"
#include "device/memory.h"

#include "util/progress.h"

CCL_NAMESPACE_BEGIN

DeviceScene::DeviceScene(Device *device)
    : bvh_nodes(device, "bvh_nodes", MEM_GLOBAL),
      bvh_leaf_nodes(device, "bvh_leaf_nodes", MEM_GLOBAL),
      object_node(device, "object_node", MEM_GLOBAL),
      prim_type(device, "prim_type", MEM_GLOBAL),
      prim_visibility(device, "prim_visibility", MEM_GLOBAL),
      prim_index(device, "prim_index", MEM_GLOBAL),
      prim_object(device, "prim_object", MEM_GLOBAL),
      prim_time(device, "prim_time", MEM_GLOBAL),
      tri_verts(device, "tri_verts", MEM_GLOBAL),
      tri_shader(device, "tri_shader", MEM_GLOBAL),
      tri_vnormal(device, "tri_vnormal", MEM_GLOBAL),
      tri_vindex(device, "tri_vindex", MEM_GLOBAL),
      tri_patch(device, "tri_patch", MEM_GLOBAL),
      tri_patch_uv(device, "tri_patch_uv", MEM_GLOBAL),
      curves(device, "curves", MEM_GLOBAL),
      curve_keys(device, "curve_keys", MEM_GLOBAL),
      curve_segments(device, "curve_segments", MEM_GLOBAL),
      patches(device, "patches", MEM_GLOBAL),
      points(device, "points", MEM_GLOBAL),
      points_shader(device, "points_shader", MEM_GLOBAL),
      objects(device, "objects", MEM_GLOBAL),
      object_motion_pass(device, "object_motion_pass", MEM_GLOBAL),
      object_motion(device, "object_motion", MEM_GLOBAL),
      object_flag(device, "object_flag", MEM_GLOBAL),
      object_volume_step(device, "object_volume_step", MEM_GLOBAL),
      object_prim_offset(device, "object_prim_offset", MEM_GLOBAL),
      camera_motion(device, "camera_motion", MEM_GLOBAL),
      attributes_map(device, "attributes_map", MEM_GLOBAL),
      attributes_float(device, "attributes_float", MEM_GLOBAL),
      attributes_float2(device, "attributes_float2", MEM_GLOBAL),
      attributes_float3(device, "attributes_float3", MEM_GLOBAL),
      attributes_float4(device, "attributes_float4", MEM_GLOBAL),
      attributes_uchar4(device, "attributes_uchar4", MEM_GLOBAL),
      light_distribution(device, "light_distribution", MEM_GLOBAL),
      lights(device, "lights", MEM_GLOBAL),
      light_background_marginal_cdf(device, "light_background_marginal_cdf", MEM_GLOBAL),
      light_background_conditional_cdf(device, "light_background_conditional_cdf", MEM_GLOBAL),
      light_tree_nodes(device, "light_tree_nodes", MEM_GLOBAL),
      light_tree_emitters(device, "light_tree_emitters", MEM_GLOBAL),
      light_to_tree(device, "light_to_tree", MEM_GLOBAL),
      object_to_tree(device, "object_to_tree", MEM_GLOBAL),
      object_lookup_offset(device, "object_lookup_offset", MEM_GLOBAL),
      triangle_to_tree(device, "triangle_to_tree", MEM_GLOBAL),
      particles(device, "particles", MEM_GLOBAL),
      svm_nodes(device, "svm_nodes", MEM_GLOBAL),
      shaders(device, "shaders", MEM_GLOBAL),
      lookup_table(device, "lookup_table", MEM_GLOBAL),
      sample_pattern_lut(device, "sample_pattern_lut", MEM_GLOBAL),
      ies_lights(device, "ies", MEM_GLOBAL)
{
  memset((void *)&data, 0, sizeof(data));
}

void DeviceScene::device_free_geometry(bool force_free)
{
  bvh_nodes.free_if_need_realloc(force_free);
  bvh_leaf_nodes.free_if_need_realloc(force_free);
  object_node.free_if_need_realloc(force_free);
  prim_type.free_if_need_realloc(force_free);
  prim_visibility.free_if_need_realloc(force_free);
  prim_index.free_if_need_realloc(force_free);
  prim_object.free_if_need_realloc(force_free);
  prim_time.free_if_need_realloc(force_free);
  tri_verts.free_if_need_realloc(force_free);
  tri_shader.free_if_need_realloc(force_free);
  tri_vnormal.free_if_need_realloc(force_free);
  tri_vindex.free_if_need_realloc(force_free);
  tri_patch.free_if_need_realloc(force_free);
  tri_patch_uv.free_if_need_realloc(force_free);
  curves.free_if_need_realloc(force_free);
  curve_keys.free_if_need_realloc(force_free);
  curve_segments.free_if_need_realloc(force_free);
  points.free_if_need_realloc(force_free);
  points_shader.free_if_need_realloc(force_free);
  patches.free_if_need_realloc(force_free);
  attributes_map.free_if_need_realloc(force_free);
  attributes_float.free_if_need_realloc(force_free);
  attributes_float2.free_if_need_realloc(force_free);
  attributes_float3.free_if_need_realloc(force_free);
  attributes_float4.free_if_need_realloc(force_free);
  attributes_uchar4.free_if_need_realloc(force_free);
}

/*
 * Clears the modified tags for all elements of the device scene
 */
void DeviceScene::device_scene_clear_modified()
{
  bvh_nodes.clear_modified();
  bvh_leaf_nodes.clear_modified();
  object_node.clear_modified();
  prim_type.clear_modified();
  prim_visibility.clear_modified();
  prim_index.clear_modified();
  prim_object.clear_modified();
  prim_time.clear_modified();
  tri_verts.clear_modified();
  tri_shader.clear_modified();
  tri_vindex.clear_modified();
  tri_patch.clear_modified();
  tri_vnormal.clear_modified();
  tri_patch_uv.clear_modified();
  curves.clear_modified();
  curve_keys.clear_modified();
  curve_segments.clear_modified();
  points.clear_modified();
  points_shader.clear_modified();
  patches.clear_modified();
  attributes_map.clear_modified();
  attributes_float.clear_modified();
  attributes_float2.clear_modified();
  attributes_float3.clear_modified();
  attributes_float4.clear_modified();
  attributes_uchar4.clear_modified();
  objects.clear_modified();
  attributes_map.clear_modified();
}

void DeviceScene::device_update_host_pointers(Device *device,
                                              DeviceScene *dscene,
                                              const GeometrySizes *p_sizes)
{
  if (dscene->tri_verts.size() > 0) {
    tri_verts.assign_mem(dscene->tri_verts);
    tri_verts.tag_modified();

    if (dscene->tri_shader.is_modified()) {
      tri_shader.assign_mem(dscene->tri_shader);
      tri_shader.tag_modified();
    }

    if (dscene->tri_vnormal.is_modified()) {
      tri_vnormal.assign_mem(dscene->tri_vnormal);
      tri_vnormal.tag_modified();
    }

    if (dscene->tri_vindex.is_modified()) {
      tri_vindex.assign_mem(dscene->tri_vindex);
      tri_vindex.tag_modified();
    }

    if (dscene->tri_patch.is_modified()) {
      tri_patch.assign_mem(dscene->tri_patch);
      tri_patch.tag_modified();
    }

    if (dscene->tri_patch_uv.is_modified()) {
      tri_patch_uv.assign_mem(dscene->tri_patch_uv);
      tri_patch_uv.tag_modified();
    }
  }

  if (dscene->curve_segments.size() > 0) {
    if (dscene->curve_keys.is_modified()) {
      curve_keys.assign_mem(dscene->curve_keys);
      curve_keys.tag_modified();
    }

    if (dscene->curves.is_modified()) {
      curves.assign_mem(dscene->curves);
      curves.tag_modified();
    }

    if (dscene->curve_segments.is_modified()) {
      curve_segments.assign_mem(dscene->curve_segments);
      curve_segments.tag_modified();
    }
  }

  if (dscene->points.size() > 0) {
    points.assign_mem(dscene->points);
    points.tag_modified();

    points_shader.assign_mem(dscene->points_shader);
    points_shader.tag_modified();
  }

  if (dscene->patches.is_modified()) {
    patches.assign_mem(dscene->patches);
    patches.tag_modified();
  }

  // Update the Attributes
  if (dscene->attributes_map.is_modified()) {
    attributes_map.assign_mem(dscene->attributes_map);
    attributes_map.tag_modified();
  }
  if (dscene->attributes_float.is_modified()) {
    attributes_float.assign_mem(dscene->attributes_float);
    attributes_float.tag_modified();
  }
  if (dscene->attributes_float2.is_modified()) {
    attributes_float2.assign_mem(dscene->attributes_float2);
    attributes_float2.tag_modified();
  }
  if (dscene->attributes_float3.is_modified()) {
    attributes_float3.assign_mem(dscene->attributes_float3);
    attributes_float3.tag_modified();
  }
  if (dscene->attributes_float4.is_modified()) {
    attributes_float4.assign_mem(dscene->attributes_float4);
    attributes_float4.tag_modified();
  }
  if (dscene->attributes_uchar4.is_modified()) {
    attributes_uchar4.assign_mem(dscene->attributes_uchar4);
    attributes_uchar4.tag_modified();
  }
  if (dscene->objects.is_modified()) {
    objects.assign_mem(dscene->objects);
    objects.tag_modified();
  }
}

/**
 * This copies the data to the devices if they have been modified
 */
void DeviceScene::device_update_mesh(Device *device,
                                     const GeometrySizes *p_sizes,
                                     Progress &progress)
{
  progress.set_status("Updating Mesh", "Copying Mesh to device");
  if (tri_verts.size() > 0) {
    tri_verts.copy_to_device_if_modified(p_sizes->vert_size, 0);
    tri_shader.copy_to_device_if_modified(p_sizes->tri_size, 0);
    tri_vnormal.copy_to_device_if_modified(p_sizes->vert_size, 0);
    tri_vindex.copy_to_device_if_modified(p_sizes->tri_size, 0);
    tri_patch.copy_to_device_if_modified(p_sizes->tri_size, 0);
    tri_patch_uv.copy_to_device_if_modified(p_sizes->vert_size, 0);
  }

  if (curve_segments.size() > 0) {
    curve_keys.copy_to_device_if_modified(p_sizes->curve_key_size, 0);
    curves.copy_to_device_if_modified(p_sizes->curve_size, 0);
    curve_segments.copy_to_device_if_modified(p_sizes->curve_segment_size, 0);
  }

  if (points.size() > 0) {
    points.copy_to_device_if_modified(p_sizes->point_size, 0);
    points_shader.copy_to_device_if_modified(p_sizes->point_size, 0);
  }

  patches.copy_to_device_if_modified(p_sizes->patch_size, 0);
}

/*
 * Copies the attribute buffer data to the devices
 */
void DeviceScene::device_update_attributes(Device *device,
                                           const AttributeSizes *sizes,
                                           Progress &progress)
{
  progress.set_status("Updating Mesh", "Copying Attributes to device");
  /* copy svm attributes to device */
  attributes_map.copy_to_device_if_modified();
  attributes_float.copy_to_device_if_modified(sizes->attr_float_size, 0);
  attributes_float2.copy_to_device_if_modified(sizes->attr_float2_size, 0);
  attributes_float3.copy_to_device_if_modified(sizes->attr_float3_size, 0);
  attributes_float4.copy_to_device_if_modified(sizes->attr_float4_size, 0);
  attributes_uchar4.copy_to_device_if_modified(sizes->attr_uchar4_size, 0);
  objects.copy_to_device_if_modified();
}

void DeviceScene::device_update_bvh2(Device *device,
                                     BVH *bvh,
                                     Progress &progress)
{
  if (bvh->params.bvh_layout == BVH_LAYOUT_BVH2) {
    BVH2 *bvh2 = static_cast<BVH2 *>(bvh);
    data.bvh.root = bvh2->pack.root_index;
    /* When using BVH2, we always have to copy/update the data as its layout is dependent on
     * the BVH's leaf nodes which may be different when the objects or vertices move. */

    if (bvh2->pack.nodes.size()) {
      bvh_nodes.assign_mem(bvh2->pack.nodes);
      bvh_nodes.copy_to_device();
    }
    if (bvh2->pack.leaf_nodes.size()) {
      bvh_leaf_nodes.assign_mem(bvh2->pack.leaf_nodes);
      bvh_leaf_nodes.copy_to_device();
    }
    if (bvh2->pack.object_node.size()) {
      object_node.assign_mem(bvh2->pack.object_node);
      object_node.copy_to_device();
    }
    if (bvh2->pack.prim_type.size()) {
      prim_type.assign_mem(bvh2->pack.prim_type);
      prim_type.copy_to_device();
    }
    if (bvh2->pack.prim_visibility.size()) {
      prim_visibility.assign_mem(bvh2->pack.prim_visibility);
      prim_visibility.copy_to_device();
    }
    if (bvh2->pack.prim_index.size()) {
      prim_index.assign_mem(bvh2->pack.prim_index);
      prim_index.copy_to_device();
    }
    if (bvh2->pack.prim_object.size()) {
      prim_object.assign_mem(bvh2->pack.prim_object);
      prim_object.copy_to_device();
    }
    if (bvh2->pack.prim_time.size()) {
      prim_time.assign_mem(bvh2->pack.prim_time);
      prim_time.copy_to_device();
    }
  }
}

CCL_NAMESPACE_END
