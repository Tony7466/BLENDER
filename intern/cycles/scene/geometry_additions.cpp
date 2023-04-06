/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

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

/*
 * Clears all tags used to indicate the the shader needs to be updated.
 */
void GeometryManager::clearShaderUpdateTags(Scene *scene)
{
  /* unset flags */
  foreach (Shader *shader, scene->shaders) {
    shader->need_update_uvs = false;
    shader->need_update_attribute = false;
    shader->need_update_displacement = false;
  }
}

/*
 * Clears all tags used to indicate the the geometry needs to be updated
 * or has been modified.
 */
void GeometryManager::clearGeometryUpdateAndModifiedTags(Scene *scene)
{
  // Clear update tags
  foreach (Geometry *geom, scene->geometry) {
    // Clear update indicators
    if (geom->is_modified() || geom->need_update_bvh_for_offset) {
      geom->need_update_rebuild = false;
      geom->need_update_bvh_for_offset = false;
    }

    // Clear modified tags
    geom->clear_modified();
    geom->attributes.clear_modified();
    if (geom->is_mesh()) {
      Mesh *mesh = static_cast<Mesh *>(geom);
      mesh->subd_attributes.clear_modified();
    }
  }
}

/*
 * Clears the modified tags for all elements of the device scene
 */
void GeometryManager::device_scene_clear_modified(DeviceScene *dscene)
{
  dscene->bvh_nodes.clear_modified();
  dscene->bvh_leaf_nodes.clear_modified();
  dscene->object_node.clear_modified();
  dscene->prim_type.clear_modified();
  dscene->prim_visibility.clear_modified();
  dscene->prim_index.clear_modified();
  dscene->prim_object.clear_modified();
  dscene->prim_time.clear_modified();
  dscene->tri_verts.clear_modified();
  dscene->tri_shader.clear_modified();
  dscene->tri_vindex.clear_modified();
  dscene->tri_patch.clear_modified();
  dscene->tri_vnormal.clear_modified();
  dscene->tri_patch_uv.clear_modified();
  dscene->curves.clear_modified();
  dscene->curve_keys.clear_modified();
  dscene->curve_segments.clear_modified();
  dscene->points.clear_modified();
  dscene->points_shader.clear_modified();
  dscene->patches.clear_modified();
  dscene->attributes_map.clear_modified();
  dscene->attributes_float.clear_modified();
  dscene->attributes_float2.clear_modified();
  dscene->attributes_float3.clear_modified();
  dscene->attributes_float4.clear_modified();
  dscene->attributes_uchar4.clear_modified();
  dscene->objects.clear_modified();
  dscene->attributes_map.clear_modified();
}





/*
 * Assigns the host pointers to the sub-devicescenes so
 * that they all have the same data sources
 */
void GeometryManager::device_update_host_pointers(Device *device,
                                                  DeviceScene *dscene,
                                                  DeviceScene *sub_dscene,
                                                  GeometrySizes *p_sizes)
{
  if (p_sizes->tri_size != 0) {
    if (dscene->tri_verts.is_modified()) {
      sub_dscene->tri_verts.assign_mem(dscene->tri_verts);
      sub_dscene->tri_verts.tag_modified();
    }
    {
      if (dscene->tri_shader.is_modified()) {
        sub_dscene->tri_shader.assign_mem(dscene->tri_shader);
        sub_dscene->tri_shader.tag_modified();
      }
    }
    {
      if (dscene->tri_vnormal.is_modified()) {
        sub_dscene->tri_vnormal.assign_mem(dscene->tri_vnormal);
        sub_dscene->tri_vnormal.tag_modified();
      }
    }
    {
      if (dscene->tri_vindex.is_modified()) {
        sub_dscene->tri_vindex.assign_mem(dscene->tri_vindex);
        sub_dscene->tri_vindex.tag_modified();
      }
    }
    {
      if (dscene->tri_patch.is_modified()) {
        sub_dscene->tri_patch.assign_mem(dscene->tri_patch);
        sub_dscene->tri_patch.tag_modified();
      }
    }
    {
      if (dscene->tri_patch_uv.is_modified()) {
        sub_dscene->tri_patch_uv.assign_mem(dscene->tri_patch_uv);
        sub_dscene->tri_patch_uv.tag_modified();
      }
    }
  }

  if (p_sizes->curve_segment_size != 0) {
    if (dscene->curve_keys.is_modified()) {
      sub_dscene->curve_keys.assign_mem(dscene->curve_keys);
      sub_dscene->curve_keys.tag_modified();
    }

    if (dscene->curves.is_modified()) {
      sub_dscene->curves.assign_mem(dscene->curves);
      sub_dscene->curves.tag_modified();
    }

    if (dscene->curve_segments.is_modified()) {
      sub_dscene->curve_segments.assign_mem(dscene->curve_segments);
    }
  }

  if (p_sizes->point_size != 0) {
    // TODO: Why does this not check the modified tag?
    sub_dscene->points.assign_mem(dscene->points);
    // sub_dscene->points.tag_modified();

    sub_dscene->points_shader.assign_mem(dscene->points_shader);
    // sub_dscene->points_shader.tag_modified();
  }

  if (p_sizes->patch_size != 0 && dscene->patches.need_realloc()) {
    sub_dscene->patches.assign_mem(dscene->patches);
  }

  // Update the Attributes
  if (dscene->attributes_map.is_modified()) {
    sub_dscene->attributes_map.assign_mem(dscene->attributes_map);
    sub_dscene->attributes_map.tag_modified();
  }
  if (dscene->attributes_float.is_modified()) {
    sub_dscene->attributes_float.assign_mem(dscene->attributes_float);
    sub_dscene->attributes_float.tag_modified();
  }
  if (dscene->attributes_float2.is_modified()) {
    sub_dscene->attributes_float2.assign_mem(dscene->attributes_float2);
    sub_dscene->attributes_float2.tag_modified();
  }
  if (dscene->attributes_float3.is_modified()) {
    sub_dscene->attributes_float3.assign_mem(dscene->attributes_float3);
    sub_dscene->attributes_float3.tag_modified();
  }
  if (dscene->attributes_float4.is_modified()) {
    sub_dscene->attributes_float4.assign_mem(dscene->attributes_float4);
    sub_dscene->attributes_float4.tag_modified();
  }
  if (dscene->attributes_uchar4.is_modified()) {
    sub_dscene->attributes_uchar4.assign_mem(dscene->attributes_uchar4);
    sub_dscene->attributes_uchar4.tag_modified();
  }
  if (dscene->objects.is_modified()) {
    sub_dscene->objects.assign_mem(dscene->objects);
    sub_dscene->objects.tag_modified();
  }
}

/*
 * Records all the geometry buffer sizes for later use
 */
void GeometryManager::geom_calc_offset(Scene *scene, GeometrySizes *p_sizes)
{
  // Zero sizes
  p_sizes->vert_size = 0;
  p_sizes->tri_size = 0;

  p_sizes->curve_size = 0;
  p_sizes->curve_key_size = 0;
  p_sizes->curve_segment_size = 0;

  p_sizes->point_size = 0;

  p_sizes->patch_size = 0;
  p_sizes->face_size = 0;
  p_sizes->corner_size = 0;

  // Write the buffer offsets to the geometries and increment the sizes
  foreach (Geometry *geom, scene->geometry) {
    bool prim_offset_changed = false;
    if (geom->geometry_type == Geometry::MESH || geom->geometry_type == Geometry::VOLUME) {
      Mesh *mesh = static_cast<Mesh *>(geom);

      prim_offset_changed = (mesh->prim_offset != p_sizes->tri_size);

      mesh->vert_offset = p_sizes->vert_size;
      mesh->prim_offset = p_sizes->tri_size;

      mesh->patch_offset = p_sizes->patch_size;
      mesh->face_offset = p_sizes->face_size;
      mesh->corner_offset = p_sizes->corner_size;

      p_sizes->vert_size += mesh->verts.size();
      // Store extra index set for motion blur
      if(mesh->get_use_motion_blur()) {
	p_sizes->tri_size += 2*mesh->num_triangles();
      } else {
	p_sizes->tri_size += mesh->num_triangles();
      }

      if (mesh->get_num_subd_faces()) {
        Mesh::SubdFace last = mesh->get_subd_face(mesh->get_num_subd_faces() - 1);
        p_sizes->patch_size += (last.ptex_offset + last.num_ptex_faces()) * 8;

        /* patch tables are stored in same array so include them in patch_size */
        if (mesh->patch_table) {
          mesh->patch_table_offset = p_sizes->patch_size;
          p_sizes->patch_size += mesh->patch_table->total_size();
        }
      }

      p_sizes->face_size += mesh->get_num_subd_faces();
      p_sizes->corner_size += mesh->subd_face_corners.size();
    }
    else if (geom->is_hair()) {
      Hair *hair = static_cast<Hair *>(geom);

      prim_offset_changed = (hair->curve_segment_offset != p_sizes->curve_segment_size);
      hair->curve_key_offset = p_sizes->curve_key_size;
      hair->curve_segment_offset = p_sizes->curve_segment_size;
      hair->prim_offset = p_sizes->curve_size;

      p_sizes->curve_size += hair->num_curves();
      p_sizes->curve_key_size += hair->get_curve_keys().size();
      p_sizes->curve_segment_size += hair->num_segments();
    }
    else if (geom->is_pointcloud()) {
      PointCloud *pointcloud = static_cast<PointCloud *>(geom);

      prim_offset_changed = (pointcloud->prim_offset != p_sizes->point_size);

      pointcloud->prim_offset = p_sizes->point_size;
      p_sizes->point_size += pointcloud->num_points();
    }
    // Used to determine if the BVH needs to be recalculated
    // as the buffer offsets have been altered.
    geom->need_update_bvh_for_offset = prim_offset_changed;
  }
}





bool GeometryManager::displacement_and_curve_shadow_transparency(
    Scene *scene,
    Device *device,
    DeviceScene *dscene,
    GeometrySizes *sizes,
    AttributeSizes *attrib_sizes,
    vector<AttributeRequestSet> &geom_attributes,
    vector<AttributeRequestSet> &object_attributes,
    vector<AttributeSet> &object_attribute_values,
    Progress &progress)
{
  scoped_callback_timer timer([scene](double time) {
    if (scene->update_stats) {
      scene->update_stats->geometry.times.add_entry({"device_update (displacement)", time});
    }
  });
  /* Signal for shaders like displacement not to do ray tracing. */
  dscene->data.bvh.bvh_layout = BVH_LAYOUT_NONE;
  scene->object_manager->device_update_flags(device, dscene, scene, progress, false);

  bool displacement_done = false;
  bool curve_shadow_transparency_done = false;
  {
    // Need to upload the attribute and mesh buffers for dispacement.
    // Evaluate these on a single device (anyone will do, so use the first)
    {
      // Could break this out across all the devices as
      // the results are read back to the host. For now, the computations
      // are done on the first device.
      DeviceScene *sub_dscene = scene->dscenes.front();
      Device *sub_device = sub_dscene->tri_verts.device;
      {
        scoped_callback_timer timer([scene](double time) {
          if (scene->update_stats) {
            scene->update_stats->geometry.times.add_entry(
                {"device_update (displacement: copy meshes to device)", time});
          }
        });
        device_update_host_pointers(sub_device, dscene, sub_dscene, sizes);
        device_update_attributes(sub_device, sub_dscene, attrib_sizes, progress);
        device_update_mesh(sub_device, sub_dscene, sizes, progress);
      }
        /* Copy constant data needed by shader evaluation. */
        sub_device->const_copy_to("data", &dscene->data, sizeof(dscene->data));

      foreach (Geometry *geom, scene->geometry) {
        /* Update images needed for true displacement. */
        {
          scoped_callback_timer timer([scene](double time) {
            if (scene->update_stats) {
              scene->update_stats->geometry.times.add_entry(
                  {"device_update (displacement: load images)", time});
            }
          });
          device_update_displacement_images(sub_device, scene, progress);
        }

        if (geom->is_modified()) {
          if (geom->is_mesh()) {
            Mesh *mesh = static_cast<Mesh *>(geom);
            if (displace(sub_device, scene, mesh, progress)) {
              displacement_done = true;
            }
          }
          else if (geom->geometry_type == Geometry::HAIR) {
            Hair *hair = static_cast<Hair *>(geom);
            if (hair->update_shadow_transparency(sub_device, scene, progress)) {
              curve_shadow_transparency_done = true;
            }
          }
        }
      }
    }

    // Some host side code here as the mesh and attributes need to be
    // recalculated after displacement and shadow transparency
    /* Device re-update after displacement. */
    if (displacement_done || curve_shadow_transparency_done) {
      scoped_callback_timer timer([scene](double time) {
        if (scene->update_stats) {
          scene->update_stats->geometry.times.add_entry(
              {"device_update (displacement: attributes)", time});
        }
      });

      // Need to redo host side filling out the attribute and mesh buffers as these may have
      // changed. Hair adds a new attribute buffer and displace updates the mesh.
      geom_calc_offset(scene, sizes);
      gather_attributes(
          scene, geom_attributes, object_attributes, object_attribute_values, attrib_sizes);
      device_free(device, dscene, false);
      device_update_attributes_preprocess(device,
                                          dscene,
                                          scene,
                                          geom_attributes,
                                          object_attributes,
                                          object_attribute_values,
                                          attrib_sizes,
                                          progress);
      device_update_mesh_preprocess(device, dscene, scene, sizes, progress);
    }
  }

  return scene->object_manager->need_flags_update;
}
CCL_NAMESPACE_END
