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
 * Packs the attribute buffers and records the sizes and offsets using
 * the attribute sets
 */
bool GeometryManager::device_update_attributes_preprocess(
    Device *device,
    DeviceScene *dscene,
    Scene *scene,
    vector<AttributeRequestSet> &geom_attributes,
    vector<AttributeRequestSet> &object_attributes,
    vector<AttributeSet> &object_attribute_values,
    AttributeSizes *sizes,
    Progress &progress)
{
  bool update_obj_offsets = false;

  progress.set_status("Updating Mesh", "Computing attributes");

  // SHOULD NOT ALLOC ONLY ALLOC IF MORE SPACE IS NEEDED
  dscene->attributes_float.alloc(sizes->attr_float_size);
  dscene->attributes_float2.alloc(sizes->attr_float2_size);
  dscene->attributes_float3.alloc(sizes->attr_float3_size);
  dscene->attributes_float4.alloc(sizes->attr_float4_size);
  dscene->attributes_uchar4.alloc(sizes->attr_uchar4_size);

  /* The order of those flags needs to match that of AttrKernelDataType. */
  const bool attributes_need_realloc[AttrKernelDataType::NUM] = {
      dscene->attributes_float.need_realloc(),
      dscene->attributes_float2.need_realloc(),
      dscene->attributes_float3.need_realloc(),
      dscene->attributes_float4.need_realloc(),
      dscene->attributes_uchar4.need_realloc(),
  };

  size_t attr_float_offset = 0;
  size_t attr_float2_offset = 0;
  size_t attr_float3_offset = 0;
  size_t attr_float4_offset = 0;
  size_t attr_uchar4_offset = 0;

  /* Fill in attributes. */
  for (size_t i = 0; i < scene->geometry.size(); i++) {
    Geometry *geom = scene->geometry[i];
    AttributeRequestSet &attributes = geom_attributes[i];

    /* todo: we now store std and name attributes from requests even if
     * they actually refer to the same mesh attributes, optimize */
    foreach (AttributeRequest &req, attributes.requests) {
      Attribute *attr = geom->attributes.find(req);

      if (attr) {
        /* force a copy if we need to reallocate all the data */
        attr->modified |= attributes_need_realloc[Attribute::kernel_type(*attr)];
      }

      update_attribute_element_offset(geom,
                                      dscene->attributes_float,
                                      attr_float_offset,
                                      dscene->attributes_float2,
                                      attr_float2_offset,
                                      dscene->attributes_float3,
                                      attr_float3_offset,
                                      dscene->attributes_float4,
                                      attr_float4_offset,
                                      dscene->attributes_uchar4,
                                      attr_uchar4_offset,
                                      attr,
                                      ATTR_PRIM_GEOMETRY,
                                      req.type,
                                      req.desc);

      if (geom->is_mesh()) {
        Mesh *mesh = static_cast<Mesh *>(geom);
        Attribute *subd_attr = mesh->subd_attributes.find(req);

        if (subd_attr) {
          /* force a copy if we need to reallocate all the data */
          subd_attr->modified |= attributes_need_realloc[Attribute::kernel_type(*subd_attr)];
        }

        update_attribute_element_offset(mesh,
                                        dscene->attributes_float,
                                        attr_float_offset,
                                        dscene->attributes_float2,
                                        attr_float2_offset,
                                        dscene->attributes_float3,
                                        attr_float3_offset,
                                        dscene->attributes_float4,
                                        attr_float4_offset,
                                        dscene->attributes_uchar4,
                                        attr_uchar4_offset,
                                        subd_attr,
                                        ATTR_PRIM_SUBD,
                                        req.subd_type,
                                        req.subd_desc);
      }

      // if (progress.get_cancel())
      //   return update_obj_offsets;
    }
  }

  for (size_t i = 0; i < scene->objects.size(); i++) {
    Object *object = scene->objects[i];
    AttributeRequestSet &attributes = object_attributes[i];
    AttributeSet &values = object_attribute_values[i];

    foreach (AttributeRequest &req, attributes.requests) {
      Attribute *attr = values.find(req);

      if (attr) {
        attr->modified |= attributes_need_realloc[Attribute::kernel_type(*attr)];
      }

      update_attribute_element_offset(object->geometry,
                                      dscene->attributes_float,
                                      attr_float_offset,
                                      dscene->attributes_float2,
                                      attr_float2_offset,
                                      dscene->attributes_float3,
                                      attr_float3_offset,
                                      dscene->attributes_float4,
                                      attr_float4_offset,
                                      dscene->attributes_uchar4,
                                      attr_uchar4_offset,
                                      attr,
                                      ATTR_PRIM_GEOMETRY,
                                      req.type,
                                      req.desc);

      /* object attributes don't care about subdivision */
      req.subd_type = req.type;
      req.subd_desc = req.desc;
    }
  }

  /* create attribute lookup maps */
  if (scene->shader_manager->use_osl())
    update_osl_globals(device, scene);

  update_svm_attributes(device, dscene, scene, geom_attributes, object_attributes);

  update_obj_offsets = scene->object_manager->device_update_geom_offsets(device, dscene, scene);

  return update_obj_offsets;
}

/**
 * Packs the geometry data into the device scene. That is it fills out
 * the geometry buffers
 */
void GeometryManager::device_update_mesh_preprocess(
    Device *device, DeviceScene *dscene, Scene *scene, GeometrySizes *p_sizes, Progress &progress)
{
  /* Fill in all the arrays. */
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
        // if (progress.get_cancel())
        //   return;
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
        // if (progress.get_cancel())
        //   return;
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
        // if (progress.get_cancel())
        //   return;
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

        // if (progress.get_cancel())
        //   return;
      }
    }
  }
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

/*
 * Records the sizes of the attribute buffers
 */
static void update_attribute_element_size(Geometry *geom,
                                          Attribute *mattr,
                                          AttributePrimitive prim,
                                          size_t *attr_float_size,
                                          size_t *attr_float2_size,
                                          size_t *attr_float3_size,
                                          size_t *attr_float4_size,
                                          size_t *attr_uchar4_size)
{
  if (mattr) {
    size_t size = mattr->element_size(geom, prim);

    if (mattr->element == ATTR_ELEMENT_VOXEL) {
      /* pass */
    }
    else if (mattr->element == ATTR_ELEMENT_CORNER_BYTE) {
      *attr_uchar4_size += size;
    }
    else if (mattr->type == TypeDesc::TypeFloat) {
      *attr_float_size += size;
    }
    else if (mattr->type == TypeFloat2) {
      *attr_float2_size += size;
    }
    else if (mattr->type == TypeDesc::TypeMatrix) {
      *attr_float4_size += size * 4;
    }
    else if (mattr->type == TypeFloat4 || mattr->type == TypeRGBA) {
      *attr_float4_size += size;
    }
    else {
      *attr_float3_size += size;
    }
  }
}

/*
 * Records all the attribute buffer sizes for all the attribute buffers for later use
 */
void GeometryManager::attrib_calc_sizes(Scene *scene,
                                        AttributeSizes *p_sizes,
                                        vector<AttributeRequestSet> &geom_attributes,
                                        vector<AttributeRequestSet> &object_attributes,
                                        vector<AttributeSet> &object_attribute_values)
{
  p_sizes->attr_float_size = 0;
  p_sizes->attr_float2_size = 0;
  p_sizes->attr_float3_size = 0;
  p_sizes->attr_float4_size = 0;
  p_sizes->attr_uchar4_size = 0;

  for (size_t i = 0; i < scene->geometry.size(); i++) {
    Geometry *geom = scene->geometry[i];
    AttributeRequestSet &attributes = geom_attributes[i];
    foreach (AttributeRequest &req, attributes.requests) {
      Attribute *attr = geom->attributes.find(req);

      update_attribute_element_size(geom,
                                    attr,
                                    ATTR_PRIM_GEOMETRY,
                                    &(p_sizes->attr_float_size),
                                    &(p_sizes->attr_float2_size),
                                    &(p_sizes->attr_float3_size),
                                    &(p_sizes->attr_float4_size),
                                    &(p_sizes->attr_uchar4_size));

      if (geom->is_mesh()) {
        Mesh *mesh = static_cast<Mesh *>(geom);
        Attribute *subd_attr = mesh->subd_attributes.find(req);

        update_attribute_element_size(mesh,
                                      subd_attr,
                                      ATTR_PRIM_SUBD,
                                      &(p_sizes->attr_float_size),
                                      &(p_sizes->attr_float2_size),
                                      &(p_sizes->attr_float3_size),
                                      &(p_sizes->attr_float4_size),
                                      &(p_sizes->attr_uchar4_size));
      }
    }
  }

  for (size_t i = 0; i < scene->objects.size(); i++) {
    Object *object = scene->objects[i];

    foreach (Attribute &attr, object_attribute_values[i].attributes) {
      update_attribute_element_size(object->geometry,
                                    &attr,
                                    ATTR_PRIM_GEOMETRY,
                                    &(p_sizes->attr_float_size),
                                    &(p_sizes->attr_float2_size),
                                    &(p_sizes->attr_float3_size),
                                    &(p_sizes->attr_float4_size),
                                    &(p_sizes->attr_uchar4_size));
    }
  }
}

/*
 * Records the set of attributes used by the objects
 */
void GeometryManager::gather_attributes(Scene *scene,
                                        vector<AttributeRequestSet> &geom_attributes,
                                        vector<AttributeRequestSet> &object_attributes,
                                        vector<AttributeSet> &object_attribute_values,
                                        AttributeSizes *sizes)
{
  geom_attributes.clear();
  object_attributes.clear();
  object_attribute_values.clear();

  /* gather per mesh requested attributes. as meshes may have multiple
   * shaders assigned, this merges the requested attributes that have
   * been set per shader by the shader manager */
  geom_attributes.resize(scene->geometry.size());

  for (size_t i = 0; i < scene->geometry.size(); i++) {
    Geometry *geom = scene->geometry[i];

    geom->index = i;
    scene->need_global_attributes(geom_attributes[i]);

    foreach (Node *node, geom->get_used_shaders()) {
      Shader *shader = static_cast<Shader *>(node);
      geom_attributes[i].add(shader->attributes);
    }

    if (geom->is_hair() && static_cast<Hair *>(geom)->need_shadow_transparency()) {
      geom_attributes[i].add(ATTR_STD_SHADOW_TRANSPARENCY);
    }
  }

  /* convert object attributes to use the same data structures as geometry ones */
  object_attributes.resize(scene->objects.size());
  object_attribute_values.reserve(scene->objects.size());

  for (size_t i = 0; i < scene->objects.size(); i++) {
    Object *object = scene->objects[i];
    Geometry *geom = object->geometry;
    size_t geom_idx = geom->index;

    assert(geom_idx < scene->geometry.size() && scene->geometry[geom_idx] == geom);

    object_attribute_values.push_back(AttributeSet(geom, ATTR_PRIM_GEOMETRY));

    AttributeRequestSet &geom_requests = geom_attributes[geom_idx];
    AttributeRequestSet &attributes = object_attributes[i];
    AttributeSet &values = object_attribute_values[i];

    for (size_t j = 0; j < object->attributes.size(); j++) {
      ParamValue &param = object->attributes[j];

      /* add attributes that are requested and not already handled by the mesh */
      if (geom_requests.find(param.name()) && !geom->attributes.find(param.name())) {
        attributes.add(param.name());

        Attribute *attr = values.add(param.name(), param.type(), ATTR_ELEMENT_OBJECT);
        assert(param.datasize() == attr->buffer.size());
        memcpy(attr->buffer.data(), param.data(), param.datasize());
      }
    }
  }

  /* Geometry attributes are stored in a single array per data type. Here determine the
   * sizes of those buffers.
   */
  attrib_calc_sizes(scene, sizes, geom_attributes, object_attributes, object_attribute_values);
}

void GeometryManager::device_update_bvh2(Device *device,
                                         DeviceScene *dscene,
                                         Scene *scene,
                                         Progress &progress)
{
  BVH *bvh = scene->bvh;
  if (bvh->params.bvh_layout == BVH_LAYOUT_BVH2) {
    BVH2 *bvh2 = static_cast<BVH2 *>(bvh);

    /* When using BVH2, we always have to copy/update the data as its layout is dependent on
     * the BVH's leaf nodes which may be different when the objects or vertices move. */

    if (bvh2->pack.nodes.size()) {
      dscene->bvh_nodes.assign_mem(bvh2->pack.nodes);
      dscene->bvh_nodes.copy_to_device();
    }
    if (bvh2->pack.leaf_nodes.size()) {
      dscene->bvh_leaf_nodes.assign_mem(bvh2->pack.leaf_nodes);
      dscene->bvh_leaf_nodes.copy_to_device();
    }
    if (bvh2->pack.object_node.size()) {
      dscene->object_node.assign_mem(bvh2->pack.object_node);
      dscene->object_node.copy_to_device();
    }
    if (bvh2->pack.prim_type.size()) {
      dscene->prim_type.assign_mem(bvh2->pack.prim_type);
      dscene->prim_type.copy_to_device();
    }
    if (bvh2->pack.prim_visibility.size()) {
      dscene->prim_visibility.assign_mem(bvh2->pack.prim_visibility);
      dscene->prim_visibility.copy_to_device();
    }
    if (bvh2->pack.prim_index.size()) {
      dscene->prim_index.assign_mem(bvh2->pack.prim_index);
      dscene->prim_index.copy_to_device();
    }
    if (bvh2->pack.prim_object.size()) {
      dscene->prim_object.assign_mem(bvh2->pack.prim_object);
      dscene->prim_object.copy_to_device();
    }
    if (bvh2->pack.prim_time.size()) {
      dscene->prim_time.assign_mem(bvh2->pack.prim_time);
      dscene->prim_time.copy_to_device();
    }
  }
}

void GeometryManager::device_update_bvh_postprocess(Device *device,
                                                    DeviceScene *dscene,
                                                    Scene *scene,
                                                    Progress &progress)
{
  BVH *bvh = scene->bvh;

  const bool has_bvh2_layout = (bvh->params.bvh_layout == BVH_LAYOUT_BVH2);

  //PackedBVH pack;
  if (has_bvh2_layout) {
    BVH2 *bvh2 = static_cast<BVH2 *>(scene->bvh);
    //pack = std::move(static_cast<BVH2 *>(bvh)->pack);
    dscene->data.bvh.root = bvh2->pack.root_index;
  }
  else {
    //pack.root_index = -1;
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
  const BVHLayout bvh_layout = BVHParams::best_bvh_layout(params->bvh_layout,
                                                          device->get_bvh_layout_mask());
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
      const BVHLayout bvh_layout = BVHParams::best_bvh_layout(params->bvh_layout,
                                                              device->get_bvh_layout_mask());

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
      // sub_bvh->geometry = bvh->geometry;
      // sub_bvh->objects = bvh->objects;
    }
  }
  else {
    progress->set_status(msg, "Building BVH");
    // Don't redo the setup if this is not a sub-bvh
    if (sub_bvh != bvh) {
      // Yes, so setup the device specific sub_bvh in the multi-bvh.
      BVHParams bparams = bvh->params;
      // Set the layout to the correct one for the device
      if (bvh->params.bvh_layout == BVH_LAYOUT_MULTI_OPTIX)
        bparams.bvh_layout = BVH_LAYOUT_OPTIX;
      else if (bvh->params.bvh_layout == BVH_LAYOUT_MULTI_METAL)
        bparams.bvh_layout = BVH_LAYOUT_METAL;
      else if (bvh->params.bvh_layout == BVH_LAYOUT_MULTI_OPTIX_EMBREE)
        bparams.bvh_layout = device->info.type == DEVICE_OPTIX ? BVH_LAYOUT_OPTIX :
                                                                 BVH_LAYOUT_EMBREE;
      else if (bvh->params.bvh_layout == BVH_LAYOUT_MULTI_METAL_EMBREE)
        bparams.bvh_layout = device->info.type == DEVICE_METAL ? BVH_LAYOUT_METAL :
                                                                 BVH_LAYOUT_EMBREE;
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
  bparams.bvh_layout = BVHParams::best_bvh_layout(scene->params.bvh_layout,
                                                  device->get_bvh_layout_mask());
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
