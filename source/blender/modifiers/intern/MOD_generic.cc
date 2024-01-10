/* SPDX-FileCopyrightText: 2005 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup modifiers
 */

#include "BLI_utildefines.h"
#include "BLI_task.h"
#include "BLI_math_matrix.h"
#include "BLI_math_vector_types.hh"
#include "BLI_math_matrix_types.hh"
#include "BLI_string_ref.hh"

#include "BLT_translation.h"

#include "DNA_screen_types.h"
#include "DNA_material_types.h"

#include "BKE_curves.hh"
#include "BKE_customdata.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_geometry_set.hh"
#include "BKE_screen.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "ED_grease_pencil.hh"

#include "MOD_modifiertypes.hh"
#include "MOD_ui_common.hh"

#include "RNA_prototypes.h"

#include "DEG_depsgraph.hh"

static int ensure_material(Main &bmain, Object &ob, const blender::StringRef name, const float stroke_rgba_linear[4])
{
  int index;
  Material *ma = BKE_grease_pencil_object_material_ensure_by_name(
      &bmain, &ob, DATA_(name.begin()), &index);

  copy_v4_v4(ma->gp_style->stroke_rgba, stroke_rgba_linear);

  ma->gp_style->flag |= GP_MATERIAL_STROKE_SHOW;

  return index;
}


// unused yet
static int generate_gpencil_strokes(GreasePencil &gp){
    int vert_num=2;

    //const int material = ensure_material()

    gp.layers_for_write()={};
    blender::bke::greasepencil::Layer &layer = gp.add_layer("New Layer");
    GreasePencilFrame *frame = layer.add_frame(0,0,0);

    gp.drawings()={};
    gp.add_empty_drawings(1);
    blender::bke::greasepencil::Drawing &drawing=*gp.get_editable_drawing_at(&layer,0);
    
    blender::Array<blender::float3> positions={{0,0,0},{1,1,1}};
    blender::Array<float> radii={2.0,1.0};
    blender::Array<float> opacities={0.8,1.0};
    blender::Array<int> offsets={0,2};
    blender::Array<int> materials={0};
    float matrix[4][4];
    unit_m4(matrix);
                                        
    const blender::bke::CurvesGeometry &curves =
        blender::ed::greasepencil::create_drawing_data(positions.as_span(),
            radii.as_span(), opacities.as_span(), offsets.as_span(), materials.as_span(),
            blender::float4x4(matrix));

    curves.tag_topology_changed();
    drawing.strokes_for_write() = curves;

    return 1;
}

static void modify_geometry_set(ModifierData *md,
                                const ModifierEvalContext *ctx,
                                blender::bke::GeometrySet *geometry_set)
{
    GreasePencil *gp=geometry_set->get_grease_pencil_for_write();
    if (!gp){ return; }

    if(generate_gpencil_strokes(*gp)){
        DEG_id_tag_update(&gp->id, ID_RECALC_GEOMETRY);
    }

    //geometry_set->replace_grease_pencil(gp);
}


static void panel_draw(const bContext * /*C*/, Panel *panel)
{
  uiLayout *layout = panel->layout;

  PointerRNA ob_ptr;
  PointerRNA *ptr = modifier_panel_get_property_pointers(panel, &ob_ptr);

  uiLayoutSetPropSep(layout, true);
  
  uiItemL(layout, "What's going on", 0);

  modifier_panel_end(layout, ptr);
}

static void panel_register(ARegionType *region_type)
{
  PanelType *panel_type = modifier_panel_register(
      region_type, eModifierType_Hello, panel_draw);
}

ModifierTypeInfo modifierType_Hello = {
    /*idname*/ "Hello Modifier",
    /*name*/ N_("Hello Modifier"),
    /*struct_name*/ "HelloModifierData",
    /*struct_size*/ sizeof(HelloModifierData),
    /*srna*/ &RNA_HelloModifier,
    /*type*/ ModifierTypeType::Nonconstructive,
    /*flags*/ eModifierTypeFlag_AcceptsGreasePencil,
    /*icon*/ ICON_GREASEPENCIL,

    /*copy_data*/ nullptr,

    /*deform_verts*/ nullptr,
    /*deform_matrices*/ nullptr,
    /*deform_verts_EM*/ nullptr,
    /*deform_matrices_EM*/ nullptr,
    /*modify_mesh*/ nullptr,
    /*modify_geometry_set*/ modify_geometry_set,

    /*init_data*/ nullptr,
    /*required_data_mask*/ nullptr,
    /*free_data*/ nullptr,
    /*is_disabled*/ nullptr,
    /*update_depsgraph*/ nullptr,
    /*depends_on_time*/ nullptr,
    /*depends_on_normals*/ nullptr,
    /*foreach_ID_link*/ nullptr,
    /*foreach_tex_link*/ nullptr,
    /*free_runtime_data*/ nullptr,
    /*panel_register*/ panel_register,
    /*blend_write*/ nullptr,
    /*blend_read*/ nullptr,
};
