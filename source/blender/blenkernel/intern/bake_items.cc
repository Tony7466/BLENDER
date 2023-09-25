/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_bake_items.hh"
#include "BKE_bake_items_serialize.hh"
#include "BKE_curves.hh"
#include "BKE_idprop.hh"
#include "BKE_instances.hh"
#include "BKE_lib_id.h"
#include "BKE_material.h"
#include "BKE_mesh.hh"
#include "BKE_pointcloud.h"

#include "BLI_endian_defines.h"
#include "BLI_endian_switch.h"
#include "BLI_math_matrix_types.hh"
#include "BLI_path_util.h"

#include "DNA_material_types.h"

#include "RNA_access.hh"
#include "RNA_enum_types.hh"

namespace blender::bke::bake {

using namespace io::serialize;
using DictionaryValuePtr = std::shared_ptr<DictionaryValue>;

GeometryBakeItem::GeometryBakeItem(GeometrySet geometry) : geometry(std::move(geometry)) {}

static void remove_materials(Material ***materials, short *materials_num)
{
  MEM_SAFE_FREE(*materials);
  *materials_num = 0;
}

static void store_materials_as_id_properties(ID &id)
{
  Material ***materials = BKE_id_material_array_p(&id);
  short *materials_num = BKE_id_material_len_p(&id);
  if (*materials_num == 0) {
    return;
  }

  IDProperty *materials_prop = IDP_NewIDPArray(".materials");
  for (const int i : IndexRange(*materials_num)) {
    const Material *material = (*materials)[i];
    IDProperty *material_prop = bke::idprop::create_group(std::to_string(i)).release();
    if (material != nullptr) {
      IDP_AddToGroup(material_prop, IDP_NewString(material->id.name + 2, "id_name"));
      if (material->id.lib != nullptr) {
        IDP_AddToGroup(material_prop, IDP_NewString(material->id.lib->id.name + 2, "lib_name"));
      }
    }
    IDP_AppendArray(materials_prop, material_prop);
    /* IDP_AppendArray does a shallow copy. */
    MEM_freeN(material_prop);
  }
  IDProperty *id_properties = IDP_EnsureProperties(&id);
  IDP_ReplaceInGroup(id_properties, materials_prop);

  remove_materials(materials, materials_num);
}

/**
 * Removes parts of the geometry that can't be stored in the simulation state:
 * - Anonymous attributes can't be stored because it is not known which of them will or will not be
 *   used in the future.
 * - Materials can't be stored directly, because they are linked ID data blocks that can't be
 *   restored from baked data currently.
 */
void GeometryBakeItem::cleanup_geometry(GeometrySet &main_geometry)
{
  main_geometry.ensure_owns_all_data();
  main_geometry.modify_geometry_sets([&](GeometrySet &geometry) {
    if (Mesh *mesh = geometry.get_mesh_for_write()) {
      mesh->attributes_for_write().remove_anonymous();
      store_materials_as_id_properties(mesh->id);
    }
    if (Curves *curves = geometry.get_curves_for_write()) {
      curves->geometry.wrap().attributes_for_write().remove_anonymous();
      store_materials_as_id_properties(curves->id);
    }
    if (PointCloud *pointcloud = geometry.get_pointcloud_for_write()) {
      pointcloud->attributes_for_write().remove_anonymous();
      store_materials_as_id_properties(pointcloud->id);
    }
    if (bke::Instances *instances = geometry.get_instances_for_write()) {
      instances->attributes_for_write().remove_anonymous();
    }
    geometry.keep_only_during_modify({GeometryComponent::Type::Mesh,
                                      GeometryComponent::Type::Curve,
                                      GeometryComponent::Type::PointCloud,
                                      GeometryComponent::Type::Instance});
  });
}

PrimitiveBakeItem::PrimitiveBakeItem(const CPPType &type, const void *value) : type_(type)
{
  value_ = MEM_mallocN_aligned(type.size(), type.alignment(), __func__);
  type.copy_construct(value, value_);
}

PrimitiveBakeItem::~PrimitiveBakeItem()
{
  type_.destruct(value_);
  MEM_freeN(value_);
}

StringBakeItem::StringBakeItem(std::string value) : value_(std::move(value)) {}

BakeStateRef::BakeStateRef(const BakeState &bake_state)
{
  this->items_by_id.reserve(bake_state.items_by_id.size());
  for (auto item : bake_state.items_by_id.items()) {
    this->items_by_id.add_new(item.key, item.value.get());
  }
}

}  // namespace blender::bke::bake
