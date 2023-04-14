/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup modifiers
 */

#include <vector>

#include "BKE_geometry_set.hh"
#include "BKE_lib_id.h"
#include "BKE_lib_query.h"
#include "BKE_mesh_runtime.h"
#include "BKE_mesh_wrapper.h"
#include "BKE_modifier.h"
#include "BKE_object.h"
#include "BKE_volume.h"

#include "BLT_translation.h"

#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_object_types.h"
#include "DNA_screen_types.h"
#include "DNA_volume_types.h"

#include "DEG_depsgraph.h"

#include "GEO_mesh_to_volume.hh"

#include "UI_interface.h"
#include "UI_resources.h"

#include "BLO_read_write.h"

#include "MEM_guardedalloc.h"

#include "MOD_modifiertypes.h"
#include "MOD_ui_common.h"

#include "BLI_index_range.hh"
#include "BLI_math_matrix_types.hh"
#include "BLI_span.hh"

#include "RNA_access.h"
#include "RNA_prototypes.h"

static void initData(ModifierData *md)
{
  MeshToVolumeModifierData *mvmd = reinterpret_cast<MeshToVolumeModifierData *>(md);
  mvmd->object = nullptr;
  mvmd->resolution_mode = MESH_TO_VOLUME_RESOLUTION_MODE_VOXEL_AMOUNT;
  mvmd->band_units = MESH_TO_VOLUME_UNIT_LOCAL;
  mvmd->voxel_size = 0.1f;
  mvmd->voxel_amount = 32;
  mvmd->interior_band_width = 0.1f;
  mvmd->interior_band_voxels = 3;
  mvmd->density = 1.0f;
}

static void updateDepsgraph(ModifierData *md, const ModifierUpdateDepsgraphContext *ctx)
{
  MeshToVolumeModifierData *mvmd = reinterpret_cast<MeshToVolumeModifierData *>(md);
  DEG_add_depends_on_transform_relation(ctx->node, "Mesh to Volume Modifier");
  if (mvmd->object) {
    DEG_add_object_relation(
        ctx->node, mvmd->object, DEG_OB_COMP_GEOMETRY, "Mesh to Volume Modifier");
    DEG_add_object_relation(
        ctx->node, mvmd->object, DEG_OB_COMP_TRANSFORM, "Mesh to Volume Modifier");
  }
}

static void foreachIDLink(ModifierData *md, Object *ob, IDWalkFunc walk, void *userData)
{
  MeshToVolumeModifierData *mvmd = reinterpret_cast<MeshToVolumeModifierData *>(md);
  walk(userData, ob, (ID **)&mvmd->object, IDWALK_CB_NOP);
}

static void panel_draw(const bContext * /*C*/, Panel *panel)
{
  uiLayout *layout = panel->layout;

  PointerRNA *ptr = modifier_panel_get_property_pointers(panel, nullptr);
  MeshToVolumeModifierData *mvmd = static_cast<MeshToVolumeModifierData *>(ptr->data);

  uiLayoutSetPropSep(layout, true);

  uiItemR(layout, ptr, "object", 0, nullptr, ICON_NONE);
  uiItemR(layout, ptr, "density", 0, nullptr, ICON_NONE);

  {
    uiLayout *col = uiLayoutColumn(layout, false);
    uiItemR(col, ptr, "resolution_mode", 0, nullptr, ICON_NONE);
    if (mvmd->resolution_mode == MESH_TO_VOLUME_RESOLUTION_MODE_VOXEL_AMOUNT) {
      uiItemR(col, ptr, "voxel_amount", 0, nullptr, ICON_NONE);
    }
    else {
      uiItemR(col, ptr, "voxel_size", 0, nullptr, ICON_NONE);
    }
  }
  {
    uiLayout *col = uiLayoutColumn(layout, false);
    uiItemR(col, ptr, "band_units", 0, nullptr, ICON_NONE);
    if (mvmd->band_units == MESH_TO_VOLUME_UNIT_LOCAL) {
      uiItemR(col, ptr, "interior_band_width", 0, nullptr, ICON_NONE);
    }
    else {
      uiItemR(col, ptr, "interior_band_voxels", 0, nullptr, ICON_NONE);
    }
  }
  modifier_panel_end(layout, ptr);
}

static void panelRegister(ARegionType *region_type)
{
  modifier_panel_register(region_type, eModifierType_MeshToVolume, panel_draw);
}

static Volume *mesh_to_volume(ModifierData *md,
                              const ModifierEvalContext *ctx,
                              Volume *input_volume)
{
#ifdef WITH_OPENVDB
  using namespace blender;

  MeshToVolumeModifierData *mvmd = reinterpret_cast<MeshToVolumeModifierData *>(md);
  Object *object_to_convert = mvmd->object;

  if (object_to_convert == nullptr) {
    return input_volume;
  }
  Mesh *mesh = BKE_modifier_get_evaluated_mesh_from_evaluated_object(object_to_convert);
  if (mesh == nullptr) {
    return input_volume;
  }
  BKE_mesh_wrapper_ensure_mdata(mesh);

  const float4x4 mesh_to_own_object_space_transform = float4x4(ctx->object->world_to_object) *
                                                      float4x4(object_to_convert->object_to_world);
  const float volume_simplify = BKE_volume_simplify_factor(ctx->depsgraph);
  if (volume_simplify == 0.0f) {
    return input_volume;
  }

  auto unit_mode = (MeshToVolumeModifierBandUnits)mvmd->band_units;
  float interior_band_width;

  if (unit_mode == MESH_TO_VOLUME_UNIT_LOCAL) {
    interior_band_width = mvmd->interior_band_width;
    if (interior_band_width < 1e-5f) {
      return input_volume;
    }
  }
  else {
    const int voxels = mvmd->interior_band_voxels;
    if (voxels < 1) {
      return input_volume;
    }
    interior_band_width = float(voxels);
  }

  geometry::MeshToVolumeSettings settings{/* voxels */ 0,
                                          /* voxel_size */ 0.0f,
                                          /* interior_band_width */ interior_band_width,
                                          /* exterior_band_width */ 0.0f,
                                          /* density */ mvmd->density,
                                          /* simplify */ volume_simplify,
                                          /* fill_volume */ false,
                                          /* use_world_space_units */ unit_mode ==
                                              MESH_TO_VOLUME_UNIT_LOCAL,
                                          /* convert_to_fog */ true,
                                          /* unsigned_distance */ false};

  auto bounds_fn = [&](float3 &r_min, float3 &r_max) {
    const BoundBox *bb = BKE_object_boundbox_get(mvmd->object);
    r_min = bb->vec[0];
    r_max = bb->vec[6];
  };

  auto mode = (MeshToVolumeModifierResolutionMode)mvmd->resolution_mode;
  if (mode == MESH_TO_VOLUME_RESOLUTION_MODE_VOXEL_AMOUNT) {
    settings.voxels = mvmd->voxel_amount;
    if (settings.voxels <= 0) {
      return input_volume;
    }
    settings.voxel_size = geometry::volume_compute_voxel_size(
        settings, bounds_fn, mesh_to_own_object_space_transform);
  }
  else if (mode == MESH_TO_VOLUME_RESOLUTION_MODE_VOXEL_SIZE) {
    settings.voxel_size = mvmd->voxel_size;
  }

  if (settings.voxel_size < 1e-5f) {
    /* The voxel size is too small. */
    return input_volume;
  }

  /* Create a new volume. */
  Volume *volume;
  if (input_volume == nullptr) {
    volume = static_cast<Volume *>(BKE_id_new_nomain(ID_VO, "Volume"));
  }
  else {
    volume = BKE_volume_new_for_eval(input_volume);
  }

  /* Convert mesh to grid and add to volume. */
  geometry::volume_grid_add_from_mesh(
      volume, "density", mesh, mesh_to_own_object_space_transform, settings);

  return volume;

#else
  UNUSED_VARS(md);
  BKE_modifier_set_error(ctx->object, md, "Compiled without OpenVDB");
  return input_volume;
#endif
}

static void modifyGeometrySet(ModifierData *md,
                              const ModifierEvalContext *ctx,
                              GeometrySet *geometry_set)
{
  Volume *input_volume = geometry_set->get_volume_for_write();
  Volume *result_volume = mesh_to_volume(md, ctx, input_volume);
  if (result_volume != input_volume) {
    geometry_set->replace_volume(result_volume);
  }
}

ModifierTypeInfo modifierType_MeshToVolume = {
    /*name*/ N_("Mesh to Volume"),
    /*structName*/ "MeshToVolumeModifierData",
    /*structSize*/ sizeof(MeshToVolumeModifierData),
    /*srna*/ &RNA_MeshToVolumeModifier,
    /*type*/ eModifierTypeType_Constructive,
    /*flags*/ static_cast<ModifierTypeFlag>(0),
    /*icon*/ ICON_VOLUME_DATA, /* TODO: Use correct icon. */

    /*copyData*/ BKE_modifier_copydata_generic,

    /*deformVerts*/ nullptr,
    /*deformMatrices*/ nullptr,
    /*deformVertsEM*/ nullptr,
    /*deformMatricesEM*/ nullptr,
    /*modifyMesh*/ nullptr,
    /*modifyGeometrySet*/ modifyGeometrySet,

    /*initData*/ initData,
    /*requiredDataMask*/ nullptr,
    /*freeData*/ nullptr,
    /*isDisabled*/ nullptr,
    /*updateDepsgraph*/ updateDepsgraph,
    /*dependsOnTime*/ nullptr,
    /*dependsOnNormals*/ nullptr,
    /*foreachIDLink*/ foreachIDLink,
    /*foreachTexLink*/ nullptr,
    /*freeRuntimeData*/ nullptr,
    /*panelRegister*/ panelRegister,
    /*blendWrite*/ nullptr,
    /*blendRead*/ nullptr,
};
