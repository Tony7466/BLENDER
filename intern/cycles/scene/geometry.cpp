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
#include "scene/osl.h"
#include "scene/pointcloud.h"
#include "scene/scene.h"
#include "scene/shader.h"
#include "scene/shader_nodes.h"
#include "scene/stats.h"
#include "scene/volume.h"

#include "subd/patch_table.h"
#include "subd/split.h"

#ifdef WITH_OSL
#  include "kernel/osl/globals.h"
#endif

#include "util/foreach.h"
#include "util/log.h"
#include "util/progress.h"
#include "util/task.h"

CCL_NAMESPACE_BEGIN

/* Geometry */

NODE_ABSTRACT_DEFINE(Geometry)
{
  NodeType *type = NodeType::add("geometry_base", NULL);

  SOCKET_UINT(motion_steps, "Motion Steps", 3);
  SOCKET_BOOLEAN(use_motion_blur, "Use Motion Blur", false);
  SOCKET_NODE_ARRAY(used_shaders, "Shaders", Shader::get_node_type());

  return type;
}

Geometry::Geometry(const NodeType *node_type, const Type type)
    : Node(node_type), geometry_type(type), attributes(this, ATTR_PRIM_GEOMETRY)
{
  need_update_rebuild = false;
  need_update_bvh_for_offset = false;

  transform_applied = false;
  transform_negative_scaled = false;
  transform_normal = transform_identity();
  bounds = BoundBox::empty;

  has_volume = false;
  has_surface_bssrdf = false;

  bvh = NULL;
  attr_map_offset = 0;
  prim_offset = 0;
}

Geometry::~Geometry()
{
  dereference_all_used_nodes();
  delete bvh;
}

void Geometry::clear(bool preserve_shaders)
{
  if (!preserve_shaders)
    used_shaders.clear();

  transform_applied = false;
  transform_negative_scaled = false;
  transform_normal = transform_identity();
  tag_modified();
}

float Geometry::motion_time(int step) const
{
  return (motion_steps > 1) ? 2.0f * step / (motion_steps - 1) - 1.0f : 0.0f;
}

int Geometry::motion_step(float time) const
{
  if (motion_steps > 1) {
    int attr_step = 0;

    for (int step = 0; step < motion_steps; step++) {
      float step_time = motion_time(step);
      if (step_time == time) {
        return attr_step;
      }

      /* Center step is stored in a separate attribute. */
      if (step != motion_steps / 2) {
        attr_step++;
      }
    }
  }

  return -1;
}

bool Geometry::need_build_bvh(BVHLayout layout) const
{
  return is_instanced() || layout == BVH_LAYOUT_OPTIX || layout == BVH_LAYOUT_MULTI_OPTIX ||
         layout == BVH_LAYOUT_METAL || layout == BVH_LAYOUT_MULTI_OPTIX_EMBREE ||
         layout == BVH_LAYOUT_MULTI_METAL || layout == BVH_LAYOUT_MULTI_METAL_EMBREE ||
         layout == BVH_LAYOUT_HIPRT || layout == BVH_LAYOUT_MULTI_HIPRT ||
         layout == BVH_LAYOUT_MULTI_HIPRT_EMBREE || layout == BVH_LAYOUT_EMBREEGPU ||
         layout == BVH_LAYOUT_MULTI_EMBREEGPU || layout == BVH_LAYOUT_MULTI_EMBREEGPU_EMBREE;
}

bool Geometry::is_instanced() const
{
  /* Currently we treat subsurface objects as instanced.
   *
   * While it might be not very optimal for ray traversal, it avoids having
   * duplicated BVH in the memory, saving quite some space.
   */
  return !transform_applied || has_surface_bssrdf;
}

bool Geometry::has_true_displacement() const
{
  foreach (Node *node, used_shaders) {
    Shader *shader = static_cast<Shader *>(node);
    if (shader->has_displacement && shader->get_displacement_method() != DISPLACE_BUMP) {
      return true;
    }
  }

  return false;
}

bool Geometry::has_motion_blur() const
{
  return (use_motion_blur && attributes.find(ATTR_STD_MOTION_VERTEX_POSITION));
}

void Geometry::tag_update(Scene *scene, bool rebuild)
{
  if (rebuild) {
    need_update_rebuild = true;
    scene->light_manager->tag_update(scene, LightManager::MESH_NEED_REBUILD);
  }
  else {
    foreach (Node *node, used_shaders) {
      Shader *shader = static_cast<Shader *>(node);
      if (shader->emission_sampling != EMISSION_SAMPLING_NONE) {
        scene->light_manager->tag_update(scene, LightManager::EMISSIVE_MESH_MODIFIED);
        break;
      }
    }
  }

  scene->geometry_manager->tag_update(scene, GeometryManager::GEOMETRY_MODIFIED);
}

void Geometry::tag_bvh_update(bool rebuild)
{
  tag_modified();

  if (rebuild) {
    need_update_rebuild = true;
  }
}

/* Geometry Manager */

GeometryManager::GeometryManager()
{
  update_flags = UPDATE_ALL;
  need_flags_update = true;
}

GeometryManager::~GeometryManager() {}

void GeometryManager::update_osl_globals(Device *device, Scene *scene)
{
#ifdef WITH_OSL
  OSLGlobals *og = (OSLGlobals *)device->get_cpu_osl_memory();
  if (og == nullptr) {
    /* Can happen when rendering with multiple GPUs, but no CPU (in which case the name maps filled
     * below are not used anyway) */
    return;
  }

  og->object_name_map.clear();
  og->object_names.clear();

  for (size_t i = 0; i < scene->objects.size(); i++) {
    /* set object name to object index map */
    Object *object = scene->objects[i];
    og->object_name_map[object->name] = i;
    og->object_names.push_back(object->name);
  }
#else
  (void)device;
  (void)scene;
#endif
}

static void update_attribute_realloc_flags(uint32_t &device_update_flags,
                                           const AttributeSet &attributes)
{
  if (attributes.modified(AttrKernelDataType::FLOAT)) {
    device_update_flags |= ATTR_FLOAT_NEEDS_REALLOC;
  }
  if (attributes.modified(AttrKernelDataType::FLOAT2)) {
    device_update_flags |= ATTR_FLOAT2_NEEDS_REALLOC;
  }
  if (attributes.modified(AttrKernelDataType::FLOAT3)) {
    device_update_flags |= ATTR_FLOAT3_NEEDS_REALLOC;
  }
  if (attributes.modified(AttrKernelDataType::FLOAT4)) {
    device_update_flags |= ATTR_FLOAT4_NEEDS_REALLOC;
  }
  if (attributes.modified(AttrKernelDataType::UCHAR4)) {
    device_update_flags |= ATTR_UCHAR4_NEEDS_REALLOC;
  }
}

static void update_device_flags_attribute(uint32_t &device_update_flags,
                                          const AttributeSet &attributes)
{
  foreach (const Attribute &attr, attributes.attributes) {
    if (!attr.modified) {
      continue;
    }

    AttrKernelDataType kernel_type = Attribute::kernel_type(attr);

    switch (kernel_type) {
      case AttrKernelDataType::FLOAT: {
        device_update_flags |= ATTR_FLOAT_MODIFIED;
        break;
      }
      case AttrKernelDataType::FLOAT2: {
        device_update_flags |= ATTR_FLOAT2_MODIFIED;
        break;
      }
      case AttrKernelDataType::FLOAT3: {
        device_update_flags |= ATTR_FLOAT3_MODIFIED;
        break;
      }
      case AttrKernelDataType::FLOAT4: {
        device_update_flags |= ATTR_FLOAT4_MODIFIED;
        break;
      }
      case AttrKernelDataType::UCHAR4: {
        device_update_flags |= ATTR_UCHAR4_MODIFIED;
        break;
      }
      case AttrKernelDataType::NUM: {
        break;
      }
    }
  }
}

void GeometryManager::device_update_preprocess(Device *device, Scene *scene, Progress &progress)
{
  if (!need_update() && !need_flags_update) {
    return;
  }

  uint32_t device_update_flags = 0;

  scoped_callback_timer timer([scene](double time) {
    if (scene->update_stats) {
      scene->update_stats->geometry.times.add_entry({"device_update_preprocess", time});
    }
  });

  progress.set_status("Updating Meshes Flags");

  /* Update flags. */
  bool volume_images_updated = false;

  foreach (Geometry *geom, scene->geometry) {
    geom->has_volume = false;

    update_attribute_realloc_flags(device_update_flags, geom->attributes);

    if (geom->is_mesh()) {
      Mesh *mesh = static_cast<Mesh *>(geom);
      update_attribute_realloc_flags(device_update_flags, mesh->subd_attributes);
    }

    foreach (Node *node, geom->get_used_shaders()) {
      Shader *shader = static_cast<Shader *>(node);
      if (shader->has_volume) {
        geom->has_volume = true;
      }

      if (shader->has_surface_bssrdf) {
        geom->has_surface_bssrdf = true;
      }

      if (shader->need_update_uvs) {
        device_update_flags |= ATTR_FLOAT2_NEEDS_REALLOC;

        /* Attributes might need to be tessellated if added. */
        if (geom->is_mesh()) {
          Mesh *mesh = static_cast<Mesh *>(geom);
          if (mesh->need_tesselation()) {
            mesh->tag_modified();
          }
        }
      }

      if (shader->need_update_attribute) {
        device_update_flags |= ATTRS_NEED_REALLOC;

        /* Attributes might need to be tessellated if added. */
        if (geom->is_mesh()) {
          Mesh *mesh = static_cast<Mesh *>(geom);
          if (mesh->need_tesselation()) {
            mesh->tag_modified();
          }
        }
      }

      if (shader->need_update_displacement) {
        /* tag displacement related sockets as modified */
        if (geom->is_mesh()) {
          Mesh *mesh = static_cast<Mesh *>(geom);
          mesh->tag_verts_modified();
          mesh->tag_subd_dicing_rate_modified();
          mesh->tag_subd_max_level_modified();
          mesh->tag_subd_objecttoworld_modified();

          device_update_flags |= ATTRS_NEED_REALLOC;
        }
      }
    }

    /* only check for modified attributes if we do not need to reallocate them already */
    if ((device_update_flags & ATTRS_NEED_REALLOC) == 0) {
      update_device_flags_attribute(device_update_flags, geom->attributes);
      /* don't check for subd_attributes, as if they were modified, we would need to reallocate
       * anyway */
    }

    /* Re-create volume mesh if we will rebuild or refit the BVH. Note we
     * should only do it in that case, otherwise the BVH and mesh can go
     * out of sync. */
    if (geom->is_modified() && geom->geometry_type == Geometry::VOLUME) {
      /* Create volume meshes if there is voxel data. */
      if (!volume_images_updated) {
        progress.set_status("Updating Meshes Volume Bounds");
        device_update_volume_images(device, scene, progress);
        volume_images_updated = true;
      }

      Volume *volume = static_cast<Volume *>(geom);
      create_volume_mesh(scene, volume, progress);

      /* always reallocate when we have a volume, as we need to rebuild the BVH */
      device_update_flags |= DEVICE_MESH_DATA_NEEDS_REALLOC;
    }

    if (geom->is_hair()) {
      /* Set curve shape, still a global scene setting for now. */
      Hair *hair = static_cast<Hair *>(geom);
      hair->curve_shape = scene->params.hair_shape;

      if (hair->need_update_rebuild) {
        device_update_flags |= DEVICE_CURVE_DATA_NEEDS_REALLOC;
      }
      else if (hair->is_modified()) {
        device_update_flags |= DEVICE_CURVE_DATA_MODIFIED;
      }
    }

    if (geom->is_mesh()) {
      Mesh *mesh = static_cast<Mesh *>(geom);

      if (mesh->need_update_rebuild) {
        device_update_flags |= DEVICE_MESH_DATA_NEEDS_REALLOC;
      }
      else if (mesh->is_modified()) {
        device_update_flags |= DEVICE_MESH_DATA_MODIFIED;
      }
    }

    if (geom->is_pointcloud()) {
      PointCloud *pointcloud = static_cast<PointCloud *>(geom);

      if (pointcloud->need_update_rebuild) {
        device_update_flags |= DEVICE_POINT_DATA_NEEDS_REALLOC;
      }
      else if (pointcloud->is_modified()) {
        device_update_flags |= DEVICE_POINT_DATA_MODIFIED;
      }
    }
  }

  if (update_flags & (MESH_ADDED | MESH_REMOVED)) {
    device_update_flags |= DEVICE_MESH_DATA_NEEDS_REALLOC;
  }

  if (update_flags & (HAIR_ADDED | HAIR_REMOVED)) {
    device_update_flags |= DEVICE_CURVE_DATA_NEEDS_REALLOC;
  }

  if (update_flags & (POINT_ADDED | POINT_REMOVED)) {
    device_update_flags |= DEVICE_POINT_DATA_NEEDS_REALLOC;
  }

  /* tag the device arrays for reallocation or modification */
  DeviceScene *dscene = &scene->dscene;

  if (device_update_flags & (DEVICE_MESH_DATA_NEEDS_REALLOC | DEVICE_CURVE_DATA_NEEDS_REALLOC |
                             DEVICE_POINT_DATA_NEEDS_REALLOC))
  {
    delete scene->bvh;
    scene->bvh = nullptr;

    dscene->bvh_nodes.tag_realloc();
    dscene->bvh_leaf_nodes.tag_realloc();
    dscene->object_node.tag_realloc();
    dscene->prim_type.tag_realloc();
    dscene->prim_visibility.tag_realloc();
    dscene->prim_index.tag_realloc();
    dscene->prim_object.tag_realloc();
    dscene->prim_time.tag_realloc();

    if (device_update_flags & DEVICE_MESH_DATA_NEEDS_REALLOC) {
      dscene->tri_verts.tag_realloc();
      dscene->tri_vnormal.tag_realloc();
      dscene->tri_vindex.tag_realloc();
      dscene->tri_patch.tag_realloc();
      dscene->tri_patch_uv.tag_realloc();
      dscene->tri_shader.tag_realloc();
      dscene->patches.tag_realloc();
    }

    if (device_update_flags & DEVICE_CURVE_DATA_NEEDS_REALLOC) {
      dscene->curves.tag_realloc();
      dscene->curve_keys.tag_realloc();
      dscene->curve_segments.tag_realloc();
    }

    if (device_update_flags & DEVICE_POINT_DATA_NEEDS_REALLOC) {
      dscene->points.tag_realloc();
      dscene->points_shader.tag_realloc();
    }
  }

  if ((update_flags & VISIBILITY_MODIFIED) != 0) {
    dscene->prim_visibility.tag_modified();
  }

  if (device_update_flags & ATTR_FLOAT_NEEDS_REALLOC) {
    dscene->attributes_map.tag_realloc();
    dscene->attributes_float.tag_realloc();
  }
  else if (device_update_flags & ATTR_FLOAT_MODIFIED) {
    dscene->attributes_float.tag_modified();
  }

  if (device_update_flags & ATTR_FLOAT2_NEEDS_REALLOC) {
    dscene->attributes_map.tag_realloc();
    dscene->attributes_float2.tag_realloc();
  }
  else if (device_update_flags & ATTR_FLOAT2_MODIFIED) {
    dscene->attributes_float2.tag_modified();
  }

  if (device_update_flags & ATTR_FLOAT3_NEEDS_REALLOC) {
    dscene->attributes_map.tag_realloc();
    dscene->attributes_float3.tag_realloc();
  }
  else if (device_update_flags & ATTR_FLOAT3_MODIFIED) {
    dscene->attributes_float3.tag_modified();
  }

  if (device_update_flags & ATTR_FLOAT4_NEEDS_REALLOC) {
    dscene->attributes_map.tag_realloc();
    dscene->attributes_float4.tag_realloc();
  }
  else if (device_update_flags & ATTR_FLOAT4_MODIFIED) {
    dscene->attributes_float4.tag_modified();
  }

  if (device_update_flags & ATTR_UCHAR4_NEEDS_REALLOC) {
    dscene->attributes_map.tag_realloc();
    dscene->attributes_uchar4.tag_realloc();
  }
  else if (device_update_flags & ATTR_UCHAR4_MODIFIED) {
    dscene->attributes_uchar4.tag_modified();
  }

  if (device_update_flags & DEVICE_MESH_DATA_MODIFIED) {
    /* if anything else than vertices or shaders are modified, we would need to reallocate, so
     * these are the only arrays that can be updated */
    dscene->tri_verts.tag_modified();
    dscene->tri_vnormal.tag_modified();
    dscene->tri_shader.tag_modified();
  }

  if (device_update_flags & DEVICE_CURVE_DATA_MODIFIED) {
    dscene->curve_keys.tag_modified();
    dscene->curves.tag_modified();
    dscene->curve_segments.tag_modified();
  }

  if (device_update_flags & DEVICE_POINT_DATA_MODIFIED) {
    dscene->points.tag_modified();
    dscene->points_shader.tag_modified();
  }

  need_flags_update = false;
}

void GeometryManager::device_update_displacement_images(Device *device,
                                                        Scene *scene,
                                                        Progress &progress)
{
  progress.set_status("Updating Displacement Images");
  TaskPool pool;
  ImageManager *image_manager = scene->image_manager;
  set<int> bump_images;
  bool has_osl_node = false;
  foreach (Geometry *geom, scene->geometry) {
    if (geom->is_modified()) {
      /* Geometry-level check for hair shadow transparency.
       * This matches the logic in the `Hair::update_shadow_transparency()`, avoiding access to
       * possible non-loaded images. */
      bool need_shadow_transparency = false;
      if (geom->geometry_type == Geometry::HAIR) {
        Hair *hair = static_cast<Hair *>(geom);
        need_shadow_transparency = hair->need_shadow_transparency();
      }

      foreach (Node *node, geom->get_used_shaders()) {
        Shader *shader = static_cast<Shader *>(node);
        const bool is_true_displacement = (shader->has_displacement &&
                                           shader->get_displacement_method() != DISPLACE_BUMP);
        if (!is_true_displacement && !need_shadow_transparency) {
          continue;
        }
        foreach (ShaderNode *node, shader->graph->nodes) {
          if (node->special_type == SHADER_SPECIAL_TYPE_OSL) {
            has_osl_node = true;
          }
          if (node->special_type != SHADER_SPECIAL_TYPE_IMAGE_SLOT) {
            continue;
          }

          ImageSlotTextureNode *image_node = static_cast<ImageSlotTextureNode *>(node);
          for (int i = 0; i < image_node->handle.num_tiles(); i++) {
            const int slot = image_node->handle.svm_slot(i);
            if (slot != -1) {
              bump_images.insert(slot);
            }
          }
        }
      }
    }
  }

#ifdef WITH_OSL
  /* If any OSL node is used for displacement, it may reference a texture. But it's
   * unknown which ones, so have to load them all. */
  if (has_osl_node) {
    OSLShaderManager::osl_image_slots(device, image_manager, bump_images);
  }
#endif

  foreach (int slot, bump_images) {
    pool.push(function_bind(
        &ImageManager::device_update_slot, image_manager, device, scene, slot, &progress));
  }
  pool.wait_work();
}

void GeometryManager::device_update_volume_images(Device *device, Scene *scene, Progress &progress)
{
  progress.set_status("Updating Volume Images");
  TaskPool pool;
  ImageManager *image_manager = scene->image_manager;
  set<int> volume_images;

  foreach (Geometry *geom, scene->geometry) {
    if (!geom->is_modified()) {
      continue;
    }

    foreach (Attribute &attr, geom->attributes.attributes) {
      if (attr.element != ATTR_ELEMENT_VOXEL) {
        continue;
      }

      ImageHandle &handle = attr.data_voxel();
      /* We can build directly from OpenVDB data structures, no need to
       * load such images early. */
      if (!handle.vdb_loader()) {
        const int slot = handle.svm_slot();
        if (slot != -1) {
          volume_images.insert(slot);
        }
      }
    }
  }

  foreach (int slot, volume_images) {
    pool.push(function_bind(
        &ImageManager::device_update_slot, image_manager, device, scene, slot, &progress));
  }
  pool.wait_work();
}

/*
 * Updates the vertex normals and adds the undisplaced attributes to the
 * mesh if they are not already present for all geometries. It marks any
 * geometry that must be rebuilt. It also determines if any displacement
 * or shadow transparancy occurs in the scene.
 */
void GeometryManager::pretess_disp_normal_and_vertices_setup(Device *device,
                                                        Scene *scene,
                                                        bool &true_displacement_used,
                                                        bool &curve_shadow_transparency_used,
                                                        size_t &total_tess_needed)
{
  scoped_callback_timer timer([scene](double time) {
    if (scene->update_stats) {
      scene->update_stats->geometry.times.add_entry({"device_update (normals)", time});
    }
  });

  const BVHLayout bvh_layout = BVHParams::best_bvh_layout(
      scene->params.bvh_layout, device->get_bvh_layout_mask(scene->dscene.data.kernel_features));

  // Calculate and/or gather mesh vertex normals and undisplaced vertices
  // also determines if tesselation, displacement or shadow transparency is needed.
  foreach (Geometry *geom, scene->geometry) {
    if (geom->is_modified()) {
      if ((geom->geometry_type == Geometry::MESH || geom->geometry_type == Geometry::VOLUME)) {
        Mesh *mesh = static_cast<Mesh *>(geom);

        /* Update normals. */
	mesh->add_vertex_normals();

        if (mesh->need_attribute(scene, ATTR_STD_POSITION_UNDISPLACED)) {
          mesh->add_undisplaced();
        }
        /* Test if we need tessellation. */
        if (mesh->need_tesselation()) {
          total_tess_needed++;
        }
        /* Test if we need displacement. */
        if (mesh->has_true_displacement()) {
          true_displacement_used = true;
        }
      }
      else if (geom->geometry_type == Geometry::HAIR) {
        Hair *hair = static_cast<Hair *>(geom);
        if (hair->need_shadow_transparency()) {
          curve_shadow_transparency_used = true;
        }
      }
    }
    if (geom->need_update_bvh_for_offset) {
      /* Need to rebuild BVH in OptiX, since refit only allows modified mesh data there */
      const bool has_optix_bvh = bvh_layout == BVH_LAYOUT_OPTIX ||
                                 bvh_layout == BVH_LAYOUT_MULTI_OPTIX ||
                                 bvh_layout == BVH_LAYOUT_MULTI_OPTIX_EMBREE;
      geom->need_update_rebuild |= has_optix_bvh;
      geom->need_update_bvh_for_offset = true;
    }
    else {
      geom->need_update_bvh_for_offset = false;
    }
  }
}

/*
 * Uploads the mesh data to the device and then builds or refits the BVH
 * using the uploaded data.
 */
void GeometryManager::device_data_xfer_and_bvh_update(SceneTimes *times,
                                                      Scene *scene,
						      Device *device,
                                                      DeviceScene *dscene,
                                                      const BVHLayout bvh_layout,
                                                      size_t num_bvh,
                                                      bool can_refit,
                                                      bool need_update_scene_bvh,
                                                      Progress &progress)
{
  /* Assign the host_pointers to the sub_dscene so that they access
     the correct data */
  if(dscene != &(scene->dscene)) {
    dscene->device_update_host_pointers(device, &(scene->dscene), &(scene->geom_sizes));
  }
  /* Upload geometry and attribute buffers to the device */
  {
    scoped_callback_timer timer([scene, times](double time) {
      if (scene->update_stats) {
        /* Save copy mesh to device duration for later logging */
        times->mesh = time;
      }
    });
    dscene->device_update_mesh(device, &(scene->geom_sizes), progress);
  }

  {
    scoped_callback_timer timer([scene, times](double time) {
      if (scene->update_stats) {
        times->attrib = time;
      }
    });
    dscene->device_update_attributes(device, &(scene->attrib_sizes), progress);
  }

  dscene->device_scene_clear_modified();
  device_init_update_bvh(scene);
  {
    scoped_callback_timer timer([scene, times](double time) {
      if (scene->update_stats) {
        times->object_bvh = time;
      }
    });
    TaskPool pool;
    
    size_t i = 0;
    /* Build the Object BVHs */
    foreach (Geometry *geom, scene->geometry) {
      if (geom->is_modified() || geom->need_update_bvh_for_offset) {
        pool.push(function_bind(
				&Geometry::compute_bvh, geom, device, dscene, &scene->params, &progress, i, num_bvh));
	//geom->compute_bvh(device, dscene, &scene->params, &progress, i, num_bvh);
        if (geom->need_build_bvh(bvh_layout)) {
          i++;
        }
      }
    }
    TaskPool::Summary summary;
    pool.wait_work(&summary);
    VLOG_WORK << "Objects BVH build pool statistics:\n" << summary.full_report();
  }

  if(need_update_scene_bvh) {
    scoped_callback_timer timer([scene, times](double time) {
      if (scene->update_stats) {
        times->scene_bvh = time;
      }
    });
    /* Build the scene BVH */
    device_update_bvh(device, dscene, scene, can_refit, 1, 1, progress);
    dscene->device_update_bvh2(device, scene->bvh, progress);
  }
}

/*
 * Calculates the bounds for any modified geometry and
 * then updates the objects bounds from the geometry.
 */
void GeometryManager::update_object_bounds(Scene *scene)
{
  Scene::MotionType need_motion = scene->need_motion();
  bool motion_blur = need_motion == Scene::MOTION_BLUR;

  scoped_callback_timer timer([scene](double time) {
    if (scene->update_stats) {
      scene->update_stats->geometry.times.add_entry({"device_update (compute bounds)", time});
    }
  });
  foreach (Geometry *geom, scene->geometry) {
    if (geom->is_modified()) {
      geom->compute_bounds();
    }
  }
  foreach (Object *object, scene->objects) {
    object->compute_bounds(motion_blur);
  }
}

/*
 * Tesselates any modified object that requires it.
 */
void GeometryManager::tesselate(Scene *scene, size_t total_tess_needed, Progress &progress)
{
  /* Tessellate meshes that are using subdivision */
  if (total_tess_needed) {
    scoped_callback_timer timer([scene](double time) {
      if (scene->update_stats) {
        scene->update_stats->geometry.times.add_entry(
            {"device_update (adaptive subdivision)", time});
      }
    });

    Camera *dicing_camera = scene->dicing_camera;
    dicing_camera->set_screen_size(dicing_camera->get_full_width(),
                                   dicing_camera->get_full_height());
    dicing_camera->update(scene);

    int geom_count = 0;
    foreach (Geometry *geom, scene->geometry) {
      if (!(geom->is_modified() && geom->is_mesh())) {
        continue;
      }
      Mesh *mesh = static_cast<Mesh *>(geom);
      if (mesh->need_tesselation()) {
        string msg = "Tessellating ";
        if (mesh->name == "")
          msg += string_printf("%u/%u", (uint)(geom_count + 1), (uint)total_tess_needed);
        else
          msg += string_printf(
              "%s %u/%u", mesh->name.c_str(), (uint)(geom_count + 1), (uint)total_tess_needed);

        progress.set_status("Updating Mesh", msg);

        mesh->subd_params->camera = dicing_camera;
        DiagSplit dsplit(*mesh->subd_params);
        mesh->tessellate(&dsplit);
      }
      geom_count++;
    }

    if (progress.get_cancel()) {
      return;
    }
  }
}

void GeometryManager::device_update(Device *device,
                                    DeviceScene *dscene,
                                    Scene *scene,
                                    Progress &progress)
{
  if (!need_update())
    return;

  VLOG_INFO << "Total " << scene->geometry.size() << " meshes.";

  bool true_displacement_used = false;
  bool curve_shadow_transparency_used = false;
  size_t total_tess_needed = 0;

  pretess_disp_normal_and_vertices_setup(
      device, scene, true_displacement_used, curve_shadow_transparency_used, total_tess_needed);

  tesselate(scene, total_tess_needed, progress);

  geom_calc_offset(scene);

  // Gather the requests attributes for filling out the attribute and geometry buffers
  vector<AttributeRequestSet> geom_attributes(scene->geometry.size());
  vector<AttributeRequestSet> object_attributes(scene->objects.size());
  vector<AttributeSet> object_attribute_values;

  progress.set_status("Updating Mesh", "Computing attributes");
  gather_attributes(
		    scene, geom_attributes, object_attributes, object_attribute_values);
  /* Device update. */
  device_free(device, dscene, false);

  device_update_attributes_preprocess(device,
                                      dscene,
                                      scene,
                                      geom_attributes,
                                      object_attributes,
                                      object_attribute_values,
                                      progress);
  device_update_mesh_preprocess(device, dscene, scene, progress);

  /* Update displacement and hair shadow transparency. */
  if (curve_shadow_transparency_used || true_displacement_used) {
    displacement_and_curve_shadow_transparency(
        scene,
        device,
        dscene,
        geom_attributes,
        object_attributes,
        object_attribute_values,
        progress);
  }

  {
    update_object_bounds(scene);
  }
  /* Update the BVH even when there is no geometry so the kernel's BVH data is still valid,
   * especially when removing all of the objects during interactive renders.
   * Also update the BVH if the transformations change, we cannot rely on tagging the Geometry
   * as modified in this case, as we may accumulate displacement if the vertices do not also
   * change. */
  bool need_update_scene_bvh = (scene->bvh == nullptr ||
                                (update_flags & (TRANSFORM_MODIFIED | VISIBILITY_MODIFIED)) != 0);
  const BVHLayout bvh_layout = BVHParams::best_bvh_layout(
      scene->params.bvh_layout, device->get_bvh_layout_mask(dscene->data.kernel_features));
  dscene->data.bvh.bvh_layout = bvh_layout;

  size_t num_bvh = create_object_bvhs(device, dscene, scene, bvh_layout, need_update_scene_bvh);
  bool can_refit_scene_bvh = true;
  if(need_update_scene_bvh) {
    can_refit_scene_bvh = device_update_bvh_preprocess(device, dscene, scene, progress);
  }
  {
    size_t num_scenes = scene->dscenes.size();
    VLOG_INFO << "Rendering using " << num_scenes << " devices";
    /* Parallel upload the geometry data to the devices and
       calculate or refit the BVHs */
    parallel_for(size_t(0), num_scenes, [=, &progress](const size_t idx) {
      DeviceScene *sub_dscene = scene->dscenes[idx].get();
      Device *sub_device = sub_dscene->tri_verts.device;
      SceneTimes *times = &(scene->times[idx]);
      device_data_xfer_and_bvh_update(times,
                                      scene,
                                      sub_device,
                                      sub_dscene,
                                      bvh_layout,
                                      num_bvh,
                                      can_refit_scene_bvh,
                                      need_update_scene_bvh,
                                      progress);
    });
    if (need_update_scene_bvh) {
      device_update_bvh_postprocess(device, dscene, scene, progress);
    }
    if (scene->update_stats) {
      double max_mesh_time = 0.0f;
      double max_attrib_time = 0.0f;
      double max_object_bvh_time = 0.0f;
      double max_scene_bvh_time = 0.0f;
      for (size_t i = 0; i < num_scenes; i++) {
        max_mesh_time = max(max_mesh_time, scene->times[i].mesh);
        max_attrib_time = max(max_attrib_time, scene->times[i].attrib);
        max_object_bvh_time = max(max_object_bvh_time, scene->times[i].object_bvh);
	max_scene_bvh_time = max(max_scene_bvh_time, scene->times[i].scene_bvh);
      }
      scene->update_stats->geometry.times.add_entry(
          {"device_update (copy meshes to device)", max_mesh_time});
      scene->update_stats->geometry.times.add_entry(
          {"device_update (attributes)", max_attrib_time});
      scene->update_stats->geometry.times.add_entry(
          {"device_update (build object BVHs)", max_object_bvh_time});
      scene->update_stats->geometry.times.add_entry(
	  {"device_update (build scene BVH)", max_scene_bvh_time});
    }
  }

  clear_geometry_update_and_modified_tags(scene);
  clear_shader_update_tags(scene);
  update_flags = UPDATE_NONE;
  dscene->device_scene_clear_modified();
}

void GeometryManager::device_free(Device *device, DeviceScene *dscene, bool force_free)
{
  dscene->device_free_geometry(force_free);
#ifdef WITH_OSL
  OSLGlobals *og = (OSLGlobals *)device->get_cpu_osl_memory();

  if (og) {
    og->object_name_map.clear();
    og->object_names.clear();
  }
#else
  (void)device;
#endif
}

void GeometryManager::tag_update(Scene *scene, uint32_t flag)
{
  update_flags |= flag;

  /* do not tag the object manager for an update if it is the one who tagged us */
  if ((flag & OBJECT_MANAGER) == 0) {
    scene->object_manager->tag_update(scene, ObjectManager::GEOMETRY_MANAGER);
  }
}

bool GeometryManager::need_update() const
{
  return update_flags != UPDATE_NONE;
}

void GeometryManager::collect_statistics(const Scene *scene, RenderStats *stats)
{
  foreach (Geometry *geometry, scene->geometry) {
    stats->mesh.geometry.add_entry(
        NamedSizeEntry(string(geometry->name.c_str()), geometry->get_total_size_in_bytes()));
  }
}

/*
 * Clears all tags used to indicate the the shader needs to be updated.
 */
void GeometryManager::clear_shader_update_tags(Scene *scene)
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
void GeometryManager::clear_geometry_update_and_modified_tags(Scene *scene)
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
 * Records all the geometry buffer sizes for later use
 */
void GeometryManager::geom_calc_offset(Scene *scene)
{
  // Zero sizes
  GeometrySizes *p_sizes = &(scene->geom_sizes);
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

/**
 * Performs displacement and calculates shadow transparency for
 * objects that require it.
 */
bool GeometryManager::displacement_and_curve_shadow_transparency(
    Scene *scene,
    Device *device,
    DeviceScene *dscene,
    vector<AttributeRequestSet> &geom_attributes,
    vector<AttributeRequestSet> &object_attributes,
    vector<AttributeSet> &object_attribute_values,
    Progress &progress)
{
  /* Signal for shaders like displacement not to do ray tracing. */
  dscene->data.bvh.bvh_layout = BVH_LAYOUT_NONE;
  scene->object_manager->device_update_flags(device, dscene, scene, progress, false);

  bool displacement_done = false;
  bool curve_shadow_transparency_done = false;
  {
    /* Need to upload the attribute and mesh buffers for dispacement.
       Evaluate these on a single device (anyone will do, so use the first) */
    {
      /* Could break this out across all the devices as
         the results are read back to the host. For now, the computations
         are done on the first device. */
      DeviceScene *sub_dscene = scene->dscenes.front().get();
      Device *sub_device = sub_dscene->tri_verts.device;

      sub_dscene->device_update_host_pointers(sub_device, dscene, &(scene->geom_sizes));
      
      {
        scoped_callback_timer timer([scene](double time) {
          if (scene->update_stats) {
            scene->update_stats->geometry.times.add_entry(
                {"device_update (displacement: copy meshes to device)", time});
          }
        });
        sub_dscene->device_update_attributes(sub_device, &(scene->attrib_sizes), progress);
      }
      {
	scoped_callback_timer timer([scene](double time) {
	  if (scene->update_stats) {
	    scene->update_stats->geometry.times.add_entry({"displacement: copy attributes to device", time});
	  }
	});
        sub_dscene->device_update_mesh(sub_device, &(scene->geom_sizes), progress);
      }
      /* Copy constant data needed by shader evaluation. */
      sub_device->const_copy_to("data", &dscene->data, sizeof(dscene->data));

      /* Update images needed for true displacement. */
      {
        scoped_callback_timer timer([scene](double time) {
          if (scene->update_stats) {
            scene->update_stats->geometry.times.add_entry(
                {"device_update (displacement: load images)", time});
          }
        });
	/* Upload the textures to all devices as this includes bump maps
	   which are used when rendering and only uploaded here. Also
	   you can't iterate over all devices as this routine sets some
	   state which stops some images getting uploaded.
	 */
	device_update_displacement_images(device, scene, progress);
      }
      {
	scoped_callback_timer timer([scene](double time) {
	  if (scene->update_stats) {
	    scene->update_stats->geometry.times.add_entry({"device_update (displacement)", time});
	  }
	});
	foreach (Geometry *geom, scene->geometry) {
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
      sub_dscene->device_free_geometry(false);
    }

    /* Some host side code here as the mesh and attributes need to be
       recalculated after displacement and shadow transparency */
    /* Device re-update after displacement. */
    if (displacement_done || curve_shadow_transparency_done) {
      scoped_callback_timer timer([scene](double time) {
        if (scene->update_stats) {
          scene->update_stats->geometry.times.add_entry(
              {"device_update (displacement: preprocess attributes & mesh)", time});
        }
      });

      /* Need to redo host side filling out the attribute and mesh buffers as these may have
         changed. Hair adds a new attribute buffer and displace updates the mesh. */
      geom_calc_offset(scene);
      gather_attributes(scene, geom_attributes, object_attributes, object_attribute_values);
      device_free(device, dscene, false);
      device_update_attributes_preprocess(device,
                                          dscene,
                                          scene,
                                          geom_attributes,
                                          object_attributes,
                                          object_attribute_values,
                                          progress);
      device_update_mesh_preprocess(device, dscene, scene, progress);
    }
  }

  return scene->object_manager->need_flags_update;
}

CCL_NAMESPACE_END
