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

bool Geometry::need_attribute(Scene *scene, AttributeStandard std)
{
  if (std == ATTR_STD_NONE)
    return false;

  if (scene->need_global_attribute(std))
    return true;

  foreach (Node *node, used_shaders) {
    Shader *shader = static_cast<Shader *>(node);
    if (shader->attributes.find(std))
      return true;
  }

  return false;
}

bool Geometry::need_attribute(Scene * /*scene*/, ustring name)
{
  if (name == ustring())
    return false;

  foreach (Node *node, used_shaders) {
    Shader *shader = static_cast<Shader *>(node);
    if (shader->attributes.find(name))
      return true;
  }

  return false;
}

AttributeRequestSet Geometry::needed_attributes()
{
  AttributeRequestSet result;

  foreach (Node *node, used_shaders) {
    Shader *shader = static_cast<Shader *>(node);
    result.add(shader->attributes);
  }

  return result;
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
         layout == BVH_LAYOUT_MULTI_METAL || layout == BVH_LAYOUT_MULTI_METAL_EMBREE;
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

void Geometry::compute_bvh(Device *device,
                           DeviceScene *dscene,
                           SceneParams *params,
                           Progress *progress,
                           size_t n,
                           size_t total)
{
  if (progress->get_cancel())
     return;

  const BVHLayout bvh_layout = BVHParams::best_bvh_layout(params->bvh_layout,
                                                          device->get_bvh_layout_mask());
  if (need_build_bvh(bvh_layout)) {
    BVH *sub_bvh = bvh->get_device_bvh(device);
    GeometryManager::device_update_sub_bvh(
        device, dscene, bvh, sub_bvh, !need_update_rebuild, n, total, progress);
  }
}

bool Geometry::has_motion_blur() const
{
  return (use_motion_blur && attributes.find(ATTR_STD_MOTION_VERTEX_POSITION));
}

bool Geometry::has_voxel_attributes() const
{
  foreach (const Attribute &attr, attributes.attributes) {
    if (attr.element == ATTR_ELEMENT_VOXEL) {
      return true;
    }
  }

  return false;
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

/* Generate a normal attribute map entry from an attribute descriptor. */
static void emit_attribute_map_entry(AttributeMap *attr_map,
                                     size_t index,
                                     uint64_t id,
                                     TypeDesc type,
                                     const AttributeDescriptor &desc)
{
  attr_map[index].id = id;
  attr_map[index].element = desc.element;
  attr_map[index].offset = as_uint(desc.offset);

  if (type == TypeDesc::TypeFloat)
    attr_map[index].type = NODE_ATTR_FLOAT;
  else if (type == TypeDesc::TypeMatrix)
    attr_map[index].type = NODE_ATTR_MATRIX;
  else if (type == TypeFloat2)
    attr_map[index].type = NODE_ATTR_FLOAT2;
  else if (type == TypeFloat4)
    attr_map[index].type = NODE_ATTR_FLOAT4;
  else if (type == TypeRGBA)
    attr_map[index].type = NODE_ATTR_RGBA;
  else
    attr_map[index].type = NODE_ATTR_FLOAT3;

  attr_map[index].flags = desc.flags;
}

/* Generate an attribute map end marker, optionally including a link to another map.
 * Links are used to connect object attribute maps to mesh attribute maps. */
static void emit_attribute_map_terminator(AttributeMap *attr_map,
                                          size_t index,
                                          bool chain,
                                          uint chain_link)
{
  for (int j = 0; j < ATTR_PRIM_TYPES; j++) {
    attr_map[index + j].id = ATTR_STD_NONE;
    attr_map[index + j].element = chain;                     /* link is valid flag */
    attr_map[index + j].offset = chain ? chain_link + j : 0; /* link to the correct sub-entry */
    attr_map[index + j].type = 0;
    attr_map[index + j].flags = 0;
  }
}

/* Generate all necessary attribute map entries from the attribute request. */
static void emit_attribute_mapping(
    AttributeMap *attr_map, size_t index, uint64_t id, AttributeRequest &req, Geometry *geom)
{
  emit_attribute_map_entry(attr_map, index, id, req.type, req.desc);

  if (geom->is_mesh()) {
    Mesh *mesh = static_cast<Mesh *>(geom);
    if (mesh->get_num_subd_faces()) {
      emit_attribute_map_entry(attr_map, index + 1, id, req.subd_type, req.subd_desc);
    }
  }
}

void GeometryManager::update_svm_attributes(Device *,
                                            DeviceScene *dscene,
                                            Scene *scene,
                                            vector<AttributeRequestSet> &geom_attributes,
                                            vector<AttributeRequestSet> &object_attributes)
{
  /* for SVM, the attributes_map table is used to lookup the offset of an
   * attribute, based on a unique shader attribute id. */

  /* compute array stride */
  size_t attr_map_size = 0;

  for (size_t i = 0; i < scene->geometry.size(); i++) {
    Geometry *geom = scene->geometry[i];
    geom->attr_map_offset = attr_map_size;

#ifdef WITH_OSL
    size_t attr_count = 0;
    foreach (AttributeRequest &req, geom_attributes[i].requests) {
      if (req.std != ATTR_STD_NONE &&
          scene->shader_manager->get_attribute_id(req.std) != (uint64_t)req.std)
        attr_count += 2;
      else
        attr_count += 1;
    }
#else
    const size_t attr_count = geom_attributes[i].size();
#endif

    attr_map_size += (attr_count + 1) * ATTR_PRIM_TYPES;
  }

  for (size_t i = 0; i < scene->objects.size(); i++) {
    Object *object = scene->objects[i];

    /* only allocate a table for the object if it actually has attributes */
    if (object_attributes[i].size() == 0) {
      object->attr_map_offset = 0;
    }
    else {
      object->attr_map_offset = attr_map_size;
      attr_map_size += (object_attributes[i].size() + 1) * ATTR_PRIM_TYPES;
    }
  }

  if (attr_map_size == 0)
    return;

  if (!dscene->attributes_map.need_realloc()) {
    return;
  }

  /* create attribute map */
  AttributeMap *attr_map = dscene->attributes_map.alloc(attr_map_size);
  memset(attr_map, 0, dscene->attributes_map.size() * sizeof(*attr_map));

  for (size_t i = 0; i < scene->geometry.size(); i++) {
    Geometry *geom = scene->geometry[i];
    AttributeRequestSet &attributes = geom_attributes[i];

    /* set geometry attributes */
    size_t index = geom->attr_map_offset;

    foreach (AttributeRequest &req, attributes.requests) {
      uint64_t id;
      if (req.std == ATTR_STD_NONE)
        id = scene->shader_manager->get_attribute_id(req.name);
      else
        id = scene->shader_manager->get_attribute_id(req.std);

      emit_attribute_mapping(attr_map, index, id, req, geom);
      index += ATTR_PRIM_TYPES;

#ifdef WITH_OSL
      /* Some standard attributes are explicitly referenced via their standard ID, so add those
       * again in case they were added under a different attribute ID. */
      if (req.std != ATTR_STD_NONE && id != (uint64_t)req.std) {
        emit_attribute_mapping(attr_map, index, (uint64_t)req.std, req, geom);
        index += ATTR_PRIM_TYPES;
      }
#endif
    }

    emit_attribute_map_terminator(attr_map, index, false, 0);
  }

  for (size_t i = 0; i < scene->objects.size(); i++) {
    Object *object = scene->objects[i];
    AttributeRequestSet &attributes = object_attributes[i];

    /* set object attributes */
    if (attributes.size() > 0) {
      size_t index = object->attr_map_offset;

      foreach (AttributeRequest &req, attributes.requests) {
        uint64_t id;
        if (req.std == ATTR_STD_NONE)
          id = scene->shader_manager->get_attribute_id(req.name);
        else
          id = scene->shader_manager->get_attribute_id(req.std);

        emit_attribute_mapping(attr_map, index, id, req, object->geometry);
        index += ATTR_PRIM_TYPES;
      }

      emit_attribute_map_terminator(attr_map, index, true, object->geometry->attr_map_offset);
    }
  }

  /* copy to device */
  /* Copy moved to device_update_attributes */
  dscene->attributes_map.tag_modified();
}

/*
 * Copies the attribute data into the buffers and records
 * the offsets
 */
void GeometryManager::update_attribute_element_offset(Geometry *geom,
                                                      device_vector<float> &attr_float,
                                                      size_t &attr_float_offset,
                                                      device_vector<float2> &attr_float2,
                                                      size_t &attr_float2_offset,
                                                      device_vector<packed_float3> &attr_float3,
                                                      size_t &attr_float3_offset,
                                                      device_vector<float4> &attr_float4,
                                                      size_t &attr_float4_offset,
                                                      device_vector<uchar4> &attr_uchar4,
                                                      size_t &attr_uchar4_offset,
                                                      Attribute *mattr,
                                                      AttributePrimitive prim,
                                                      TypeDesc &type,
                                                      AttributeDescriptor &desc)
{
  if (mattr) {
    /* store element and type */
    desc.element = mattr->element;
    desc.flags = mattr->flags;
    type = mattr->type;

    /* store attribute data in arrays */
    size_t size = mattr->element_size(geom, prim);

    AttributeElement &element = desc.element;
    int &offset = desc.offset;

    if (mattr->element == ATTR_ELEMENT_VOXEL) {
      /* store slot in offset value */
      ImageHandle &handle = mattr->data_voxel();
      offset = handle.svm_slot();
    }
    else if (mattr->element == ATTR_ELEMENT_CORNER_BYTE) {
      uchar4 *data = mattr->data_uchar4();
      offset = attr_uchar4_offset;

      assert(attr_uchar4.size() >= offset + size);
      if (mattr->modified) {
        for (size_t k = 0; k < size; k++) {
          attr_uchar4[offset + k] = data[k];
        }
        attr_uchar4.tag_modified();
      }
      attr_uchar4_offset += size;
    }
    else if (mattr->type == TypeDesc::TypeFloat) {
      float *data = mattr->data_float();
      offset = attr_float_offset;

      assert(attr_float.size() >= offset + size);
      if (mattr->modified) {
        for (size_t k = 0; k < size; k++) {
          attr_float[offset + k] = data[k];
        }
        attr_float.tag_modified();
      }
      attr_float_offset += size;
    }
    else if (mattr->type == TypeFloat2) {
      float2 *data = mattr->data_float2();
      offset = attr_float2_offset;

      assert(attr_float2.size() >= offset + size);
      if (mattr->modified) {
        for (size_t k = 0; k < size; k++) {
          attr_float2[offset + k] = data[k];
        }
        attr_float2.tag_modified();
      }
      attr_float2_offset += size;
    }
    else if (mattr->type == TypeDesc::TypeMatrix) {
      Transform *tfm = mattr->data_transform();
      offset = attr_float4_offset;

      assert(attr_float4.size() >= offset + size * 3);
      if (mattr->modified) {
        for (size_t k = 0; k < size * 3; k++) {
          attr_float4[offset + k] = (&tfm->x)[k];
        }
        attr_float4.tag_modified();
      }
      attr_float4_offset += size * 3;
    }
    else if (mattr->type == TypeFloat4 || mattr->type == TypeRGBA) {
      float4 *data = mattr->data_float4();
      offset = attr_float4_offset;

      assert(attr_float4.size() >= offset + size);
      if (mattr->modified) {
        for (size_t k = 0; k < size; k++) {
          attr_float4[offset + k] = data[k];
        }
        attr_float4.tag_modified();
      }
      attr_float4_offset += size;
    }
    else {
      float3 *data = mattr->data_float3();
      offset = attr_float3_offset;

      // Records where the motion vertices are in the attribute array
      // so that they can be used later to reference the data when building
      // the BVHs.
      if (mattr->std == ATTR_STD_MOTION_VERTEX_POSITION) {
        geom->motion_key_offset = offset;
      }

      assert(attr_float3.size() >= offset + size);
      if (mattr->modified) {
        for (size_t k = 0; k < size; k++) {
          attr_float3[offset + k] = data[k];
        }
        attr_float3.tag_modified();
      }
      attr_float3_offset += size;
    }

    /* mesh vertex/curve index is global, not per object, so we sneak
     * a correction for that in here */
    if (geom->is_mesh()) {
      Mesh *mesh = static_cast<Mesh *>(geom);
      if (mesh->subdivision_type == Mesh::SUBDIVISION_CATMULL_CLARK &&
          desc.flags & ATTR_SUBDIVIDED) {
        /* Indices for subdivided attributes are retrieved
         * from patch table so no need for correction here. */
      }
      else if (element == ATTR_ELEMENT_VERTEX)
        offset -= mesh->vert_offset;
      else if (element == ATTR_ELEMENT_VERTEX_MOTION)
        offset -= mesh->vert_offset;
      else if (element == ATTR_ELEMENT_FACE) {
        if (prim == ATTR_PRIM_GEOMETRY)
          offset -= mesh->prim_offset;
        else
          offset -= mesh->face_offset;
      }
      else if (element == ATTR_ELEMENT_CORNER || element == ATTR_ELEMENT_CORNER_BYTE) {
        if (prim == ATTR_PRIM_GEOMETRY)
          offset -= 3 * mesh->prim_offset;
        else
          offset -= mesh->corner_offset;
      }
    }
    else if (geom->is_hair()) {
      Hair *hair = static_cast<Hair *>(geom);
      if (element == ATTR_ELEMENT_CURVE)
        offset -= hair->prim_offset;
      else if (element == ATTR_ELEMENT_CURVE_KEY)
        offset -= hair->curve_key_offset;
      else if (element == ATTR_ELEMENT_CURVE_KEY_MOTION)
        offset -= hair->curve_key_offset;
    }
    else if (geom->is_pointcloud()) {
      if (element == ATTR_ELEMENT_VERTEX)
        offset -= geom->prim_offset;
      else if (element == ATTR_ELEMENT_VERTEX_MOTION)
        offset -= geom->prim_offset;
    }
  }
  else {
    /* attribute not found */
    desc.element = ATTR_ELEMENT_NONE;
    desc.offset = 0;
  }
}

/*
 * Copies the attribute buffer data to the devices
 */
void GeometryManager::device_update_attributes(Device *device,
                                               DeviceScene *dscene,
                                               const AttributeSizes *sizes,
                                               Progress &progress)
{
  progress.set_status("Updating Mesh", "Copying Attributes to device");
  /* copy svm attributes to device */
  dscene->attributes_map.copy_to_device_if_modified();
  dscene->attributes_float.copy_to_device_if_modified(sizes->attr_float_size, 0);
  dscene->attributes_float2.copy_to_device_if_modified(sizes->attr_float2_size, 0);
  dscene->attributes_float3.copy_to_device_if_modified(sizes->attr_float3_size, 0);
  dscene->attributes_float4.copy_to_device_if_modified(sizes->attr_float4_size, 0);
  dscene->attributes_uchar4.copy_to_device_if_modified(sizes->attr_uchar4_size, 0);
  dscene->objects.copy_to_device_if_modified();
}

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

/* Set of flags used to help determining what data has been modified or needs reallocation, so we
 * can decide which device data to free or update. */
enum {
  DEVICE_CURVE_DATA_MODIFIED = (1 << 0),
  DEVICE_MESH_DATA_MODIFIED = (1 << 1),
  DEVICE_POINT_DATA_MODIFIED = (1 << 2),

  ATTR_FLOAT_MODIFIED = (1 << 3),
  ATTR_FLOAT2_MODIFIED = (1 << 4),
  ATTR_FLOAT3_MODIFIED = (1 << 5),
  ATTR_FLOAT4_MODIFIED = (1 << 6),
  ATTR_UCHAR4_MODIFIED = (1 << 7),

  CURVE_DATA_NEED_REALLOC = (1 << 8),
  MESH_DATA_NEED_REALLOC = (1 << 9),
  POINT_DATA_NEED_REALLOC = (1 << 10),

  ATTR_FLOAT_NEEDS_REALLOC = (1 << 11),
  ATTR_FLOAT2_NEEDS_REALLOC = (1 << 12),
  ATTR_FLOAT3_NEEDS_REALLOC = (1 << 13),
  ATTR_FLOAT4_NEEDS_REALLOC = (1 << 14),

  ATTR_UCHAR4_NEEDS_REALLOC = (1 << 15),

  ATTRS_NEED_REALLOC = (ATTR_FLOAT_NEEDS_REALLOC | ATTR_FLOAT2_NEEDS_REALLOC |
                        ATTR_FLOAT3_NEEDS_REALLOC | ATTR_FLOAT4_NEEDS_REALLOC |
                        ATTR_UCHAR4_NEEDS_REALLOC),
  DEVICE_MESH_DATA_NEEDS_REALLOC = (MESH_DATA_NEED_REALLOC | ATTRS_NEED_REALLOC),
  DEVICE_POINT_DATA_NEEDS_REALLOC = (POINT_DATA_NEED_REALLOC | ATTRS_NEED_REALLOC),
  DEVICE_CURVE_DATA_NEEDS_REALLOC = (CURVE_DATA_NEED_REALLOC | ATTRS_NEED_REALLOC),
};

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
                             DEVICE_POINT_DATA_NEEDS_REALLOC)) {
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
void GeometryManager::preTessDispNormalAndVerticesSetup(Device *device,
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

  const BVHLayout bvh_layout = BVHParams::best_bvh_layout(scene->params.bvh_layout,
                                                          device->get_bvh_layout_mask());

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
void GeometryManager::deviceDataXferAndBVHUpdate(int idx,
                                                 Scene *scene,
                                                 DeviceScene *dscene,
                                                 GeometrySizes &sizes,
                                                 AttributeSizes &attrib_sizes,
                                                 const BVHLayout bvh_layout,
                                                 size_t num_bvh,
						 bool can_refit,
						 bool need_update_scene_bvh,
                                                 Progress &progress)
{
  auto sub_dscene = scene->dscenes[idx];
  sub_dscene->data.bvh.bvh_layout = BVH_LAYOUT_NONE;
  // Get the device to use for this DeviceScene from one of the buffers
  Device *sub_device = sub_dscene->tri_verts.device;
  // Assign the host_pointers to the sub_dscene so that they access
  // the correct data
  device_update_host_pointers(sub_device, dscene, sub_dscene, &sizes);

  /* Upload geometry and attribute buffers to the device */
  {
    scoped_callback_timer timer([scene, idx](double time) {
      if (scene->update_stats) {
        // Save copy mesh to device duration for later logging
        scene->mesh_times[idx] = time;
      }
    });
    device_update_mesh(sub_device, sub_dscene, &sizes, progress);
  }

  {
    scoped_callback_timer timer([scene, idx](double time) {
      if (scene->update_stats) {
        scene->attrib_times[idx] = time;
      }
    });
    device_update_attributes(sub_device, sub_dscene, &attrib_sizes, progress);
  }

  device_scene_clear_modified(sub_dscene);
  {
    scoped_callback_timer timer([scene, idx](double time) {
      if (scene->update_stats) {
        scene->object_bvh_times[idx] = time;
      }
    });
    size_t i = 0;
    /* Build the Object BVHs */
    foreach (Geometry *geom, scene->geometry) {
      if (geom->is_modified() || geom->need_update_bvh_for_offset) {
        geom->compute_bvh(sub_device, sub_dscene, &scene->params, &progress, i, num_bvh);
        if (geom->need_build_bvh(bvh_layout)) {
          i++;
        }
      }
    }
  }

  if(need_update_scene_bvh) {
    scoped_callback_timer timer([scene, idx](double time) {
      if (scene->update_stats) {
        scene->scene_bvh_times[idx] = time;
      }
    });
    /* Build the scene BVH */
    device_update_bvh(sub_device, sub_dscene, scene, can_refit, 1, 1, progress);
    device_update_bvh2(sub_device, sub_dscene, scene, progress);
  }
}

/*
 * Calculates the bounds for any modified geometry and
 * then updates the objects bounds from the geometry.
 */
void GeometryManager::updateObjectBounds(Scene *scene)
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
 * Creates a new BVH for the geometry if it is needed otherwise
 * it determines if the BVH can be refitted. It also counts
 * the number of BVH that need to be built.
 */
size_t GeometryManager::createObjectBVHs(Device *device,
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

/*
 * Prepares scene BVH for building or refitting. Then builds or refits the scene
 * BVH for all the devices.
 */
void GeometryManager::updateSceneBVHs(Device *device,
                                      DeviceScene *dscene,
                                      Scene *scene,
                                      Progress &progress)
{
  scoped_callback_timer timer([scene](double time) {
    if (scene->update_stats) {
      scene->update_stats->geometry.times.add_entry({"device_update (build scene BVH)", time});
    }
  });

  bool can_refit = device_update_bvh_preprocess(device, dscene, scene, progress);
  foreach (auto sub_dscene, scene->dscenes) {
    Device *sub_device = sub_dscene->tri_verts.device;
    device_update_bvh(sub_device, sub_dscene, scene, can_refit, 1, 1, progress);
  }
  device_update_bvh_postprocess(device, dscene, scene, progress);
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

  preTessDispNormalAndVerticesSetup(
      device, scene, true_displacement_used, curve_shadow_transparency_used, total_tess_needed);

  tesselate(scene, total_tess_needed, progress);

  GeometrySizes sizes;
  geom_calc_offset(scene, &sizes);

  // Gather the requests attributes for filling out the attribute and geometry buffers
  vector<AttributeRequestSet> geom_attributes(scene->geometry.size());
  vector<AttributeRequestSet> object_attributes(scene->objects.size());
  vector<AttributeSet> object_attribute_values;
  AttributeSizes attrib_sizes;

  progress.set_status("Updating Mesh", "Computing attributes");
  gather_attributes(
      scene, geom_attributes, object_attributes, object_attribute_values, &attrib_sizes);
  /* Device update. */
  device_free(device, dscene, false);

  device_update_attributes_preprocess(device,
                                      dscene,
                                      scene,
                                      geom_attributes,
                                      object_attributes,
                                      object_attribute_values,
                                      &attrib_sizes,
                                      progress);
  device_update_mesh_preprocess(device, dscene, scene, &sizes, progress);

  /* Update displacement and hair shadow transparency. */
  if (curve_shadow_transparency_used || true_displacement_used) {
    displacement_and_curve_shadow_transparency(
        scene,
        device,
        dscene,
        &sizes,
        &attrib_sizes,
        geom_attributes,
        object_attributes,
        object_attribute_values,
        progress);
  }

  {
    updateObjectBounds(scene);
  }
  /* Update the BVH even when there is no geometry so the kernel's BVH data is still valid,
   * especially when removing all of the objects during interactive renders.
   * Also update the BVH if the transformations change, we cannot rely on tagging the Geometry
   * as modified in this case, as we may accumulate displacement if the vertices do not also
   * change. */
  bool need_update_scene_bvh = (scene->bvh == nullptr ||
                                (update_flags & (TRANSFORM_MODIFIED | VISIBILITY_MODIFIED)) != 0);
  const BVHLayout bvh_layout = BVHParams::best_bvh_layout(scene->params.bvh_layout,
                                                          device->get_bvh_layout_mask());
  dscene->data.bvh.bvh_layout = bvh_layout;

  size_t num_bvh = createObjectBVHs(device, dscene, scene, bvh_layout, need_update_scene_bvh);
  bool can_refit_scene_bvh = true;
  if(need_update_scene_bvh) {
    can_refit_scene_bvh = device_update_bvh_preprocess(device, dscene, scene, progress);
  }
  {
    size_t num_scenes = scene->dscenes.size();
    VLOG_INFO << "Rendering using " << num_scenes << " devices";
    // Parallel upload the geometry data to the devices and
    // calculate or refit the BVHs
    parallel_for(
        size_t(0), num_scenes, [=, this, &sizes, &attrib_sizes, &progress](const size_t idx) {
          deviceDataXferAndBVHUpdate(idx,
                                     scene,
                                     dscene,
                                     sizes,
                                     attrib_sizes,
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
        max_mesh_time = max(max_mesh_time, scene->mesh_times[i]);
        max_attrib_time = max(max_attrib_time, scene->attrib_times[i]);
        max_object_bvh_time = max(max_object_bvh_time, scene->object_bvh_times[i]);
	max_scene_bvh_time = max(max_scene_bvh_time, scene->scene_bvh_times[i]);
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

  clearGeometryUpdateAndModifiedTags(scene);
  clearShaderUpdateTags(scene);
  update_flags = UPDATE_NONE;
  device_scene_clear_modified(dscene);
}

void GeometryManager::device_free(Device *device, DeviceScene *dscene, bool force_free)
{
  dscene->bvh_nodes.free_if_need_realloc(force_free);
  dscene->bvh_leaf_nodes.free_if_need_realloc(force_free);
  dscene->object_node.free_if_need_realloc(force_free);
  dscene->prim_type.free_if_need_realloc(force_free);
  dscene->prim_visibility.free_if_need_realloc(force_free);
  dscene->prim_index.free_if_need_realloc(force_free);
  dscene->prim_object.free_if_need_realloc(force_free);
  dscene->prim_time.free_if_need_realloc(force_free);
  dscene->tri_verts.free_if_need_realloc(force_free);
  dscene->tri_shader.free_if_need_realloc(force_free);
  dscene->tri_vnormal.free_if_need_realloc(force_free);
  dscene->tri_vindex.free_if_need_realloc(force_free);
  dscene->tri_patch.free_if_need_realloc(force_free);
  dscene->tri_patch_uv.free_if_need_realloc(force_free);
  dscene->curves.free_if_need_realloc(force_free);
  dscene->curve_keys.free_if_need_realloc(force_free);
  dscene->curve_segments.free_if_need_realloc(force_free);
  dscene->points.free_if_need_realloc(force_free);
  dscene->points_shader.free_if_need_realloc(force_free);
  dscene->patches.free_if_need_realloc(force_free);
  dscene->attributes_map.free_if_need_realloc(force_free);
  dscene->attributes_float.free_if_need_realloc(force_free);
  dscene->attributes_float2.free_if_need_realloc(force_free);
  dscene->attributes_float3.free_if_need_realloc(force_free);
  dscene->attributes_float4.free_if_need_realloc(force_free);
  dscene->attributes_uchar4.free_if_need_realloc(force_free);

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

CCL_NAMESPACE_END
