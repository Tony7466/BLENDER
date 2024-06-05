# SPDX-FileCopyrightText: 2023 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

set(INC

)

set(INC_SYS

)

# Part of the `blender` binary (sources for data-files are appended).
set(SRC

)

set(LIB
)

# This section is maintained by the updating script, keep BEGIN/END comments.
# See: `make icons_geom` and the script `./release/datafiles/blender_icons_geom_update.py`.
set_property(GLOBAL PROPERTY ICON_GEOM_NAMES
  # BEGIN ICON_GEOM_NAMES
  brush.gpencil_draw.draw
  brush.gpencil_draw.erase
  brush.gpencil_draw.fill
  brush.gpencil_draw.tint
  brush.paint_texture.airbrush
  brush.paint_texture.clone
  brush.paint_texture.draw
  brush.paint_texture.fill
  brush.paint_texture.mask
  brush.paint_texture.masklort
  brush.paint_texture.multiply
  brush.paint_texture.smear
  brush.paint_texture.soften
  brush.paint_vertex.alpha
  brush.paint_vertex.average
  brush.paint_vertex.blur
  brush.paint_vertex.draw
  brush.paint_vertex.replace
  brush.paint_vertex.smear
  brush.paint_weight.average
  brush.paint_weight.blur
  brush.paint_weight.draw
  brush.paint_weight.mix
  brush.paint_weight.smear
  brush.particle.add
  brush.particle.comb
  brush.particle.cut
  brush.particle.length
  brush.particle.puff
  brush.particle.smooth
  brush.particle.weight
  brush.sculpt.blob
  brush.sculpt.boundary
  brush.sculpt.clay
  brush.sculpt.clay_strips
  brush.sculpt.clay_thumb
  brush.sculpt.cloth
  brush.sculpt.crease
  brush.sculpt.displacement_eraser
  brush.sculpt.displacement_smear
  brush.sculpt.draw
  brush.sculpt.draw_face_sets
  brush.sculpt.draw_sharp
  brush.sculpt.elastic_deform
  brush.sculpt.fill
  brush.sculpt.flatten
  brush.sculpt.grab
  brush.sculpt.inflate
  brush.sculpt.layer
  brush.sculpt.mask
  brush.sculpt.multiplane_scrape
  brush.sculpt.nudge
  brush.sculpt.paint
  brush.sculpt.pinch
  brush.sculpt.pose
  brush.sculpt.rotate
  brush.sculpt.scrape
  brush.sculpt.simplify
  brush.sculpt.smear
  brush.sculpt.smooth
  brush.sculpt.snake_hook
  brush.sculpt.thumb
  brush.sculpt.topology
  brush.uv_sculpt.grab
  brush.uv_sculpt.pinch
  brush.uv_sculpt.relax
  none
  ops.armature.bone.roll
  ops.armature.extrude_cursor
  ops.armature.extrude_move
  ops.curve.draw
  ops.curve.extrude_cursor
  ops.curve.extrude_move
  ops.curve.pen
  ops.curve.radius
  ops.curve.vertex_random
  ops.curves.sculpt_add
  ops.curves.sculpt_comb
  ops.curves.sculpt_cut
  ops.curves.sculpt_delete
  ops.curves.sculpt_density
  ops.curves.sculpt_grow_shrink
  ops.curves.sculpt_pinch
  ops.curves.sculpt_puff
  ops.curves.sculpt_slide
  ops.curves.sculpt_smooth
  ops.curves.sculpt_snake_hook
  ops.generic.cursor
  ops.generic.select
  ops.generic.select_box
  ops.generic.select_circle
  ops.generic.select_lasso
  ops.generic.select_paint
  ops.gpencil.draw
  ops.gpencil.draw.eraser
  ops.gpencil.draw.line
  ops.gpencil.draw.poly
  ops.gpencil.edit_bend
  ops.gpencil.edit_mirror
  ops.gpencil.edit_shear
  ops.gpencil.edit_to_sphere
  ops.gpencil.extrude_move
  ops.gpencil.primitive_arc
  ops.gpencil.primitive_box
  ops.gpencil.primitive_circle
  ops.gpencil.primitive_curve
  ops.gpencil.primitive_line
  ops.gpencil.primitive_polyline
  ops.gpencil.radius
  ops.gpencil.sculpt_average
  ops.gpencil.sculpt_blur
  ops.gpencil.sculpt_clone
  ops.gpencil.sculpt_grab
  ops.gpencil.sculpt_pinch
  ops.gpencil.sculpt_push
  ops.gpencil.sculpt_randomize
  ops.gpencil.sculpt_smear
  ops.gpencil.sculpt_smooth
  ops.gpencil.sculpt_strength
  ops.gpencil.sculpt_thickness
  ops.gpencil.sculpt_twist
  ops.gpencil.sculpt_weight
  ops.gpencil.stroke_cutter
  ops.gpencil.transform_fill
  ops.mesh.bevel
  ops.mesh.bisect
  ops.mesh.dupli_extrude_cursor
  ops.mesh.extrude_faces_move
  ops.mesh.extrude_manifold
  ops.mesh.extrude_region_move
  ops.mesh.extrude_region_shrink_fatten
  ops.mesh.inset
  ops.mesh.knife_tool
  ops.mesh.loopcut_slide
  ops.mesh.offset_edge_loops_slide
  ops.mesh.polybuild_hover
  ops.mesh.primitive_cone_add_gizmo
  ops.mesh.primitive_cube_add_gizmo
  ops.mesh.primitive_cylinder_add_gizmo
  ops.mesh.primitive_grid_add_gizmo
  ops.mesh.primitive_sphere_add_gizmo
  ops.mesh.primitive_torus_add_gizmo
  ops.mesh.rip
  ops.mesh.rip_edge
  ops.mesh.spin
  ops.mesh.vertices_smooth
  ops.node.links_cut
  ops.paint.eyedropper_add
  ops.paint.vertex_color_fill
  ops.paint.weight_fill
  ops.paint.weight_gradient
  ops.paint.weight_sample
  ops.paint.weight_sample_group
  ops.pose.breakdowner
  ops.pose.push
  ops.pose.relax
  ops.sculpt.border_face_set
  ops.sculpt.border_hide
  ops.sculpt.border_mask
  ops.sculpt.box_trim
  ops.sculpt.cloth_filter
  ops.sculpt.color_filter
  ops.sculpt.face_set_edit
  ops.sculpt.lasso_face_set
  ops.sculpt.lasso_hide
  ops.sculpt.lasso_mask
  ops.sculpt.lasso_trim
  ops.sculpt.line_face_set
  ops.sculpt.line_hide
  ops.sculpt.line_mask
  ops.sculpt.line_project
  ops.sculpt.line_trim
  ops.sculpt.mask_by_color
  ops.sculpt.mesh_filter
  ops.sculpt.polyline_face_set
  ops.sculpt.polyline_hide
  ops.sculpt.polyline_mask
  ops.sculpt.polyline_trim
  ops.sequencer.blade
  ops.sequencer.retime
  ops.transform.bone_envelope
  ops.transform.bone_size
  ops.transform.edge_slide
  ops.transform.push_pull
  ops.transform.resize.cage
  ops.transform.resize
  ops.transform.rotate
  ops.transform.shear
  ops.transform.shrink_fatten
  ops.transform.tilt
  ops.transform.tosphere
  ops.transform.transform
  ops.transform.translate
  ops.transform.vert_slide
  ops.transform.vertex_random
  ops.view3d.ruler
  # END ICON_GEOM_NAMES
)

data_to_c_simple(../../../../release/datafiles/bfont.pfb SRC)

if(WITH_BLENDER)
  # Blender only (not Cycles stand-alone).

  if(NOT WITH_HEADLESS)
    # Blender UI only.

    # Blend files.
    data_to_c_simple(../../../../release/datafiles/preview.blend SRC)
    data_to_c_simple(../../../../release/datafiles/preview_grease_pencil.blend SRC)
    data_to_c_simple(../../../../release/datafiles/preview_grease_pencil_legacy.blend SRC)

    # Images.
    data_to_c_simple(../../../../release/datafiles/splash.png SRC)
    data_to_c_simple(../../../../release/datafiles/alert_icons.png SRC)
    data_to_c_simple(../../../../release/datafiles/blender_logo.png SRC)
    data_to_c_simple(../../../../release/datafiles/prvicons.png SRC)

    # Brushes.
    data_to_c_simple(../../../../release/datafiles/brushicons/blob.png SRC)
    data_to_c_simple(../../../../release/datafiles/brushicons/blur.png SRC)
    data_to_c_simple(../../../../release/datafiles/brushicons/clay.png SRC)
    data_to_c_simple(../../../../release/datafiles/brushicons/claystrips.png SRC)
    data_to_c_simple(../../../../release/datafiles/brushicons/clone.png SRC)
    data_to_c_simple(../../../../release/datafiles/brushicons/crease.png SRC)
    data_to_c_simple(../../../../release/datafiles/brushicons/draw.png SRC)
    data_to_c_simple(../../../../release/datafiles/brushicons/fill.png SRC)
    data_to_c_simple(../../../../release/datafiles/brushicons/flatten.png SRC)
    data_to_c_simple(../../../../release/datafiles/brushicons/grab.png SRC)
    data_to_c_simple(../../../../release/datafiles/brushicons/inflate.png SRC)
    data_to_c_simple(../../../../release/datafiles/brushicons/layer.png SRC)
    data_to_c_simple(../../../../release/datafiles/brushicons/mask.png SRC)
    data_to_c_simple(../../../../release/datafiles/brushicons/mix.png SRC)
    data_to_c_simple(../../../../release/datafiles/brushicons/nudge.png SRC)
    data_to_c_simple(../../../../release/datafiles/brushicons/paint_select.png SRC)
    data_to_c_simple(../../../../release/datafiles/brushicons/pinch.png SRC)
    data_to_c_simple(../../../../release/datafiles/brushicons/scrape.png SRC)
    data_to_c_simple(../../../../release/datafiles/brushicons/smear.png SRC)
    data_to_c_simple(../../../../release/datafiles/brushicons/smooth.png SRC)
    data_to_c_simple(../../../../release/datafiles/brushicons/snake_hook.png SRC)
    data_to_c_simple(../../../../release/datafiles/brushicons/soften.png SRC)
    data_to_c_simple(../../../../release/datafiles/brushicons/texdraw.png SRC)
    data_to_c_simple(../../../../release/datafiles/brushicons/texfill.png SRC)
    data_to_c_simple(../../../../release/datafiles/brushicons/texmask.png SRC)
    data_to_c_simple(../../../../release/datafiles/brushicons/thumb.png SRC)
    data_to_c_simple(../../../../release/datafiles/brushicons/twist.png SRC)

    # Grease pencil sculpt.
    data_to_c_simple(../../../../release/datafiles/brushicons/gp_brush_smooth.png SRC)
    data_to_c_simple(../../../../release/datafiles/brushicons/gp_brush_thickness.png SRC)
    data_to_c_simple(../../../../release/datafiles/brushicons/gp_brush_strength.png SRC)
    data_to_c_simple(../../../../release/datafiles/brushicons/gp_brush_grab.png SRC)
    data_to_c_simple(../../../../release/datafiles/brushicons/gp_brush_push.png SRC)
    data_to_c_simple(../../../../release/datafiles/brushicons/gp_brush_twist.png SRC)
    data_to_c_simple(../../../../release/datafiles/brushicons/gp_brush_pinch.png SRC)
    data_to_c_simple(../../../../release/datafiles/brushicons/gp_brush_randomize.png SRC)
    data_to_c_simple(../../../../release/datafiles/brushicons/gp_brush_clone.png SRC)
    data_to_c_simple(../../../../release/datafiles/brushicons/gp_brush_weight.png SRC)

    data_to_c_simple(../../../../release/datafiles/brushicons/gp_brush_pencil.png SRC)
    data_to_c_simple(../../../../release/datafiles/brushicons/gp_brush_pen.png SRC)
    data_to_c_simple(../../../../release/datafiles/brushicons/gp_brush_ink.png SRC)
    data_to_c_simple(../../../../release/datafiles/brushicons/gp_brush_inknoise.png SRC)
    data_to_c_simple(../../../../release/datafiles/brushicons/gp_brush_block.png SRC)
    data_to_c_simple(../../../../release/datafiles/brushicons/gp_brush_marker.png SRC)
    data_to_c_simple(../../../../release/datafiles/brushicons/gp_brush_fill.png SRC)
    data_to_c_simple(../../../../release/datafiles/brushicons/gp_brush_airbrush.png SRC)
    data_to_c_simple(../../../../release/datafiles/brushicons/gp_brush_chisel.png SRC)
    data_to_c_simple(../../../../release/datafiles/brushicons/gp_brush_erase_soft.png SRC)
    data_to_c_simple(../../../../release/datafiles/brushicons/gp_brush_erase_hard.png SRC)
    data_to_c_simple(../../../../release/datafiles/brushicons/gp_brush_erase_stroke.png SRC)

    # Curve sculpt.
    data_to_c_simple(../../../../release/datafiles/brushicons/curves_sculpt_add.png SRC)
    data_to_c_simple(../../../../release/datafiles/brushicons/curves_sculpt_comb.png SRC)
    data_to_c_simple(../../../../release/datafiles/brushicons/curves_sculpt_cut.png SRC)
    data_to_c_simple(../../../../release/datafiles/brushicons/curves_sculpt_delete.png SRC)
    data_to_c_simple(../../../../release/datafiles/brushicons/curves_sculpt_density.png SRC)
    data_to_c_simple(../../../../release/datafiles/brushicons/curves_sculpt_grow_shrink.png SRC)
    data_to_c_simple(../../../../release/datafiles/brushicons/curves_sculpt_pinch.png SRC)
    data_to_c_simple(../../../../release/datafiles/brushicons/curves_sculpt_puff.png SRC)
    data_to_c_simple(../../../../release/datafiles/brushicons/curves_sculpt_slide.png SRC)
    data_to_c_simple(../../../../release/datafiles/brushicons/curves_sculpt_smooth.png SRC)
    data_to_c_simple(../../../../release/datafiles/brushicons/curves_sculpt_snake_hook.png SRC)

  endif()

  data_to_c_simple(../../../../release/datafiles/startup.blend SRC)
endif()

unset(ICON_NAMES)

blender_add_lib(bf_editor_datafiles "${SRC}" "${INC}" "${INC_SYS}" "${LIB}")
