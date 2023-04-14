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

/**
 * This copies the data to the devices if they have been modified
 */
void GeometryManager::device_update_mesh(Device *device,
                                         DeviceScene *dscene,
                                         const GeometrySizes *p_sizes,
                                         Progress &progress)
{
  progress.set_status("Updating Mesh", "Copying Mesh to device");
  if (p_sizes->tri_size != 0) {
    dscene->tri_verts.copy_to_device_if_modified(p_sizes->vert_size, 0);
    dscene->tri_shader.copy_to_device_if_modified(p_sizes->tri_size, 0);
    dscene->tri_vnormal.copy_to_device_if_modified(p_sizes->vert_size, 0);
    dscene->tri_vindex.copy_to_device_if_modified(p_sizes->tri_size, 0);
    dscene->tri_patch.copy_to_device_if_modified(p_sizes->tri_size, 0);
    dscene->tri_patch_uv.copy_to_device_if_modified(p_sizes->vert_size, 0);
  }

  if (p_sizes->curve_segment_size != 0) {
    dscene->curve_keys.copy_to_device_if_modified(p_sizes->curve_key_size, 0);
    dscene->curves.copy_to_device_if_modified(p_sizes->curve_size, 0);
    dscene->curve_segments.copy_to_device_if_modified(p_sizes->curve_segment_size, 0);
  }

  if (p_sizes->point_size != 0) {
    dscene->points.copy_to_device(p_sizes->point_size, 0);
    dscene->points_shader.copy_to_device(p_sizes->point_size, 0);
  }

  if (p_sizes->patch_size != 0 && dscene->patches.need_realloc()) {
    dscene->patches.copy_to_device(p_sizes->patch_size, 0);
  }
}

/**
 * Packs the geometry data into the device scene. That is it fills out
 * the geometry buffers
 */
void GeometryManager::device_update_mesh_preprocess(
    Device *device, DeviceScene *dscene, Scene *scene, Progress &progress)
{
  /* Fill in all the arrays. */
  GeometrySizes *p_sizes = &(scene->geom_sizes);
  if (p_sizes->tri_size != 0) {
    /* normals */
    progress.set_status("Updating Mesh", "Computing mesh");

    packed_float3 *tri_verts = dscene->tri_verts.alloc(p_sizes->vert_size);
    uint *tri_shader = dscene->tri_shader.alloc(p_sizes->tri_size);
    packed_float3 *vnormal = dscene->tri_vnormal.alloc(p_sizes->vert_size);
    packed_uint3 *tri_vindex = dscene->tri_vindex.alloc(p_sizes->tri_size);
    uint *tri_patch = dscene->tri_patch.alloc(p_sizes->tri_size);
    float2 *tri_patch_uv = dscene->tri_patch_uv.alloc(p_sizes->vert_size);

    const bool copy_all_data = dscene->tri_shader.need_realloc() ||
                               dscene->tri_vindex.need_realloc() ||
                               dscene->tri_vnormal.need_realloc() ||
                               dscene->tri_patch.need_realloc() ||
                               dscene->tri_patch_uv.need_realloc();

    foreach (Geometry *geom, scene->geometry) {
      if (geom->geometry_type == Geometry::MESH || geom->geometry_type == Geometry::VOLUME) {
        Mesh *mesh = static_cast<Mesh *>(geom);

        if (mesh->shader_is_modified() || mesh->smooth_is_modified() ||
            mesh->triangles_is_modified() || copy_all_data) {
          mesh->pack_shaders(scene, &tri_shader[mesh->prim_offset]);
        }

        if (mesh->verts_is_modified() || copy_all_data) {
          mesh->pack_normals(&vnormal[mesh->vert_offset]);
        }

        if (mesh->verts_is_modified() || mesh->triangles_is_modified() ||
            mesh->vert_patch_uv_is_modified() || copy_all_data) {
          mesh->pack_verts(&tri_verts[mesh->vert_offset],
                           &tri_vindex[mesh->prim_offset],
                           &tri_patch[mesh->prim_offset],
                           &tri_patch_uv[mesh->vert_offset]);
        }
        if (progress.get_cancel())
          return;
      }
    }
  }

  if (p_sizes->curve_segment_size != 0) {
    progress.set_status("Updating Mesh", "Computing curves");

    float4 *curve_keys = dscene->curve_keys.alloc(p_sizes->curve_key_size);
    KernelCurve *curves = dscene->curves.alloc(p_sizes->curve_size);
    KernelCurveSegment *curve_segments = dscene->curve_segments.alloc(p_sizes->curve_segment_size);

    const bool copy_all_data = dscene->curve_keys.need_realloc() ||
                               dscene->curves.need_realloc() ||
                               dscene->curve_segments.need_realloc();

    foreach (Geometry *geom, scene->geometry) {
      if (geom->is_hair()) {
        Hair *hair = static_cast<Hair *>(geom);

        bool curve_keys_co_modified = hair->curve_radius_is_modified() ||
                                      hair->curve_keys_is_modified();
        bool curve_data_modified = hair->curve_shader_is_modified() ||
                                   hair->curve_first_key_is_modified();

        if (!curve_keys_co_modified && !curve_data_modified && !copy_all_data) {
          continue;
        }

        hair->pack_curves(scene,
                          &curve_keys[hair->curve_key_offset],
                          &curves[hair->prim_offset],
                          &curve_segments[hair->curve_segment_offset]);
        if (progress.get_cancel())
          return;
      }
    }
  }

  if (p_sizes->point_size != 0) {
    progress.set_status("Updating Mesh", "Computing point clouds");

    float4 *points = dscene->points.alloc(p_sizes->point_size);
    uint *points_shader = dscene->points_shader.alloc(p_sizes->point_size);

    foreach (Geometry *geom, scene->geometry) {
      if (geom->is_pointcloud()) {
        PointCloud *pointcloud = static_cast<PointCloud *>(geom);
        pointcloud->pack(
            scene, &points[pointcloud->prim_offset], &points_shader[pointcloud->prim_offset]);
        if (progress.get_cancel())
          return;
      }
    }
  }

  if (p_sizes->patch_size != 0 && dscene->patches.need_realloc()) {
    progress.set_status("Updating Mesh", "Computing patches");

    uint *patch_data = dscene->patches.alloc(p_sizes->patch_size);
    foreach (Geometry *geom, scene->geometry) {
      if (geom->is_mesh()) {
        Mesh *mesh = static_cast<Mesh *>(geom);
        mesh->pack_patches(&patch_data[mesh->patch_offset]);

        if (mesh->patch_table) {
          mesh->patch_table->copy_adjusting_offsets(&patch_data[mesh->patch_table_offset],
                                                    mesh->patch_table_offset);
        }

        if (progress.get_cancel())
          return;
      }
    }
  }
}

CCL_NAMESPACE_END
