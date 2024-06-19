# SPDX-FileCopyrightText: 2023 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

set(INC
  PUBLIC ${CMAKE_CURRENT_BINARY_DIR}
  ../include
)

set(INC_SYS

)

# Part of the `blender` binary (sources for data-files are appended).
set(SRC

)

set(LIB
  PRIVATE bf::blenlib
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

    set(SVG_SRC
      ../../../../release/datafiles/icons_svg/action.svg
      ../../../../release/datafiles/icons_svg/action_tweak.svg
      ../../../../release/datafiles/icons_svg/add.svg
      ../../../../release/datafiles/icons_svg/aliased.svg
      ../../../../release/datafiles/icons_svg/align_bottom.svg
      ../../../../release/datafiles/icons_svg/align_center.svg
      ../../../../release/datafiles/icons_svg/align_flush.svg
      ../../../../release/datafiles/icons_svg/align_justify.svg
      ../../../../release/datafiles/icons_svg/align_left.svg
      ../../../../release/datafiles/icons_svg/align_middle.svg
      ../../../../release/datafiles/icons_svg/align_right.svg
      ../../../../release/datafiles/icons_svg/align_top.svg
      ../../../../release/datafiles/icons_svg/anchor_bottom.svg
      ../../../../release/datafiles/icons_svg/anchor_center.svg
      ../../../../release/datafiles/icons_svg/anchor_left.svg
      ../../../../release/datafiles/icons_svg/anchor_right.svg
      ../../../../release/datafiles/icons_svg/anchor_top.svg
      ../../../../release/datafiles/icons_svg/anim.svg
      ../../../../release/datafiles/icons_svg/anim_data.svg
      ../../../../release/datafiles/icons_svg/antialiased.svg
      ../../../../release/datafiles/icons_svg/append_blend.svg
      ../../../../release/datafiles/icons_svg/area_join.svg
      ../../../../release/datafiles/icons_svg/area_swap.svg
      ../../../../release/datafiles/icons_svg/armature_data.svg
      ../../../../release/datafiles/icons_svg/arrow_leftright.svg
      ../../../../release/datafiles/icons_svg/asset_manager.svg
      ../../../../release/datafiles/icons_svg/auto.svg
      ../../../../release/datafiles/icons_svg/automerge_off.svg
      ../../../../release/datafiles/icons_svg/automerge_on.svg
      ../../../../release/datafiles/icons_svg/axis_front.svg
      ../../../../release/datafiles/icons_svg/axis_side.svg
      ../../../../release/datafiles/icons_svg/axis_top.svg
      ../../../../release/datafiles/icons_svg/back.svg
      ../../../../release/datafiles/icons_svg/blank1.svg
      ../../../../release/datafiles/icons_svg/blender.svg
      ../../../../release/datafiles/icons_svg/boids.svg
      ../../../../release/datafiles/icons_svg/bold.svg
      ../../../../release/datafiles/icons_svg/bone_data.svg
      ../../../../release/datafiles/icons_svg/bookmarks.svg
      ../../../../release/datafiles/icons_svg/bordermove.svg
      ../../../../release/datafiles/icons_svg/brushes_all.svg
      ../../../../release/datafiles/icons_svg/brush_data.svg
      ../../../../release/datafiles/icons_svg/camera_data.svg
      ../../../../release/datafiles/icons_svg/camera_stereo.svg
      ../../../../release/datafiles/icons_svg/cancel.svg
      ../../../../release/datafiles/icons_svg/center_only.svg
      ../../../../release/datafiles/icons_svg/checkbox_dehlt.svg
      ../../../../release/datafiles/icons_svg/checkbox_hlt.svg
      ../../../../release/datafiles/icons_svg/checkmark.svg
      ../../../../release/datafiles/icons_svg/clipuv_dehlt.svg
      ../../../../release/datafiles/icons_svg/clipuv_hlt.svg
      ../../../../release/datafiles/icons_svg/collapsemenu.svg
      ../../../../release/datafiles/icons_svg/collection_new.svg
      ../../../../release/datafiles/icons_svg/color.svg
      ../../../../release/datafiles/icons_svg/color_blue.svg
      ../../../../release/datafiles/icons_svg/color_green.svg
      ../../../../release/datafiles/icons_svg/color_red.svg
      ../../../../release/datafiles/icons_svg/community.svg
      ../../../../release/datafiles/icons_svg/cone.svg
      ../../../../release/datafiles/icons_svg/console.svg
      ../../../../release/datafiles/icons_svg/constraint.svg
      ../../../../release/datafiles/icons_svg/constraint_bone.svg
      ../../../../release/datafiles/icons_svg/con_action.svg
      ../../../../release/datafiles/icons_svg/con_armature.svg
      ../../../../release/datafiles/icons_svg/con_camerasolver.svg
      ../../../../release/datafiles/icons_svg/con_childof.svg
      ../../../../release/datafiles/icons_svg/con_clampto.svg
      ../../../../release/datafiles/icons_svg/con_distlimit.svg
      ../../../../release/datafiles/icons_svg/con_floor.svg
      ../../../../release/datafiles/icons_svg/con_followpath.svg
      ../../../../release/datafiles/icons_svg/con_followtrack.svg
      ../../../../release/datafiles/icons_svg/con_kinematic.svg
      ../../../../release/datafiles/icons_svg/con_locktrack.svg
      ../../../../release/datafiles/icons_svg/con_loclike.svg
      ../../../../release/datafiles/icons_svg/con_loclimit.svg
      ../../../../release/datafiles/icons_svg/con_objectsolver.svg
      ../../../../release/datafiles/icons_svg/con_pivot.svg
      ../../../../release/datafiles/icons_svg/con_rotlike.svg
      ../../../../release/datafiles/icons_svg/con_rotlimit.svg
      ../../../../release/datafiles/icons_svg/con_samevol.svg
      ../../../../release/datafiles/icons_svg/con_shrinkwrap.svg
      ../../../../release/datafiles/icons_svg/con_sizelike.svg
      ../../../../release/datafiles/icons_svg/con_sizelimit.svg
      ../../../../release/datafiles/icons_svg/con_splineik.svg
      ../../../../release/datafiles/icons_svg/con_stretchto.svg
      ../../../../release/datafiles/icons_svg/con_trackto.svg
      ../../../../release/datafiles/icons_svg/con_transform.svg
      ../../../../release/datafiles/icons_svg/con_transform_cache.svg
      ../../../../release/datafiles/icons_svg/con_translike.svg
      ../../../../release/datafiles/icons_svg/copydown.svg
      ../../../../release/datafiles/icons_svg/copy_id.svg
      ../../../../release/datafiles/icons_svg/cube.svg
      ../../../../release/datafiles/icons_svg/current_file.svg
      ../../../../release/datafiles/icons_svg/cursor.svg
      ../../../../release/datafiles/icons_svg/curves.svg
      ../../../../release/datafiles/icons_svg/curves_data.svg
      ../../../../release/datafiles/icons_svg/curve_bezcircle.svg
      ../../../../release/datafiles/icons_svg/curve_bezcurve.svg
      ../../../../release/datafiles/icons_svg/curve_data.svg
      ../../../../release/datafiles/icons_svg/curve_ncircle.svg
      ../../../../release/datafiles/icons_svg/curve_ncurve.svg
      ../../../../release/datafiles/icons_svg/curve_path.svg
      ../../../../release/datafiles/icons_svg/decorate.svg
      ../../../../release/datafiles/icons_svg/decorate_animate.svg
      ../../../../release/datafiles/icons_svg/decorate_driver.svg
      ../../../../release/datafiles/icons_svg/decorate_keyframe.svg
      ../../../../release/datafiles/icons_svg/decorate_library_override.svg
      ../../../../release/datafiles/icons_svg/decorate_linked.svg
      ../../../../release/datafiles/icons_svg/decorate_locked.svg
      ../../../../release/datafiles/icons_svg/decorate_override.svg
      ../../../../release/datafiles/icons_svg/decorate_unlocked.svg
      ../../../../release/datafiles/icons_svg/desktop.svg
      ../../../../release/datafiles/icons_svg/disc.svg
      ../../../../release/datafiles/icons_svg/disclosure_tri_down.svg
      ../../../../release/datafiles/icons_svg/disclosure_tri_right.svg
      ../../../../release/datafiles/icons_svg/disk_drive.svg
      ../../../../release/datafiles/icons_svg/documents.svg
      ../../../../release/datafiles/icons_svg/dot.svg
      ../../../../release/datafiles/icons_svg/downarrow_hlt.svg
      ../../../../release/datafiles/icons_svg/driver.svg
      ../../../../release/datafiles/icons_svg/driver_distance.svg
      ../../../../release/datafiles/icons_svg/driver_rotational_difference.svg
      ../../../../release/datafiles/icons_svg/driver_transform.svg
      ../../../../release/datafiles/icons_svg/duplicate.svg
      ../../../../release/datafiles/icons_svg/edgesel.svg
      ../../../../release/datafiles/icons_svg/editmode_hlt.svg
      ../../../../release/datafiles/icons_svg/empty_arrows.svg
      ../../../../release/datafiles/icons_svg/empty_axis.svg
      ../../../../release/datafiles/icons_svg/empty_data.svg
      ../../../../release/datafiles/icons_svg/empty_single_arrow.svg
      ../../../../release/datafiles/icons_svg/error.svg
      ../../../../release/datafiles/icons_svg/experimental.svg
      ../../../../release/datafiles/icons_svg/export.svg
      ../../../../release/datafiles/icons_svg/external_drive.svg
      ../../../../release/datafiles/icons_svg/eyedropper.svg
      ../../../../release/datafiles/icons_svg/facesel.svg
      ../../../../release/datafiles/icons_svg/face_corner.svg
      ../../../../release/datafiles/icons_svg/face_maps.svg
      ../../../../release/datafiles/icons_svg/fake_user_off.svg
      ../../../../release/datafiles/icons_svg/fake_user_on.svg
      ../../../../release/datafiles/icons_svg/fcurve.svg
      ../../../../release/datafiles/icons_svg/fcurve_snapshot.svg
      ../../../../release/datafiles/icons_svg/ff.svg
      ../../../../release/datafiles/icons_svg/file.svg
      ../../../../release/datafiles/icons_svg/filebrowser.svg
      ../../../../release/datafiles/icons_svg/file_3d.svg
      ../../../../release/datafiles/icons_svg/file_archive.svg
      ../../../../release/datafiles/icons_svg/file_backup.svg
      ../../../../release/datafiles/icons_svg/file_blank.svg
      ../../../../release/datafiles/icons_svg/file_blend.svg
      ../../../../release/datafiles/icons_svg/file_cache.svg
      ../../../../release/datafiles/icons_svg/file_folder.svg
      ../../../../release/datafiles/icons_svg/file_font.svg
      ../../../../release/datafiles/icons_svg/file_hidden.svg
      ../../../../release/datafiles/icons_svg/file_image.svg
      ../../../../release/datafiles/icons_svg/file_movie.svg
      ../../../../release/datafiles/icons_svg/file_new.svg
      ../../../../release/datafiles/icons_svg/file_parent.svg
      ../../../../release/datafiles/icons_svg/file_refresh.svg
      ../../../../release/datafiles/icons_svg/file_script.svg
      ../../../../release/datafiles/icons_svg/file_sound.svg
      ../../../../release/datafiles/icons_svg/file_text.svg
      ../../../../release/datafiles/icons_svg/file_tick.svg
      ../../../../release/datafiles/icons_svg/file_volume.svg
      ../../../../release/datafiles/icons_svg/filter.svg
      ../../../../release/datafiles/icons_svg/fixed_size.svg
      ../../../../release/datafiles/icons_svg/folder_redirect.svg
      ../../../../release/datafiles/icons_svg/fontpreview.svg
      ../../../../release/datafiles/icons_svg/font_data.svg
      ../../../../release/datafiles/icons_svg/force_boid.svg
      ../../../../release/datafiles/icons_svg/force_charge.svg
      ../../../../release/datafiles/icons_svg/force_curve.svg
      ../../../../release/datafiles/icons_svg/force_drag.svg
      ../../../../release/datafiles/icons_svg/force_fluidflow.svg
      ../../../../release/datafiles/icons_svg/force_force.svg
      ../../../../release/datafiles/icons_svg/force_harmonic.svg
      ../../../../release/datafiles/icons_svg/force_lennardjones.svg
      ../../../../release/datafiles/icons_svg/force_magnetic.svg
      ../../../../release/datafiles/icons_svg/force_texture.svg
      ../../../../release/datafiles/icons_svg/force_turbulence.svg
      ../../../../release/datafiles/icons_svg/force_vortex.svg
      ../../../../release/datafiles/icons_svg/force_wind.svg
      ../../../../release/datafiles/icons_svg/forward.svg
      ../../../../release/datafiles/icons_svg/frame_next.svg
      ../../../../release/datafiles/icons_svg/frame_prev.svg
      ../../../../release/datafiles/icons_svg/freeze.svg
      ../../../../release/datafiles/icons_svg/fullscreen_enter.svg
      ../../../../release/datafiles/icons_svg/fullscreen_exit.svg
      ../../../../release/datafiles/icons_svg/fund.svg
      ../../../../release/datafiles/icons_svg/geometry_nodes.svg
      ../../../../release/datafiles/icons_svg/ghost_disabled.svg
      ../../../../release/datafiles/icons_svg/ghost_enabled.svg
      ../../../../release/datafiles/icons_svg/gizmo.svg
      ../../../../release/datafiles/icons_svg/gp_caps_flat.svg
      ../../../../release/datafiles/icons_svg/gp_caps_round.svg
      ../../../../release/datafiles/icons_svg/gp_multiframe_editing.svg
      ../../../../release/datafiles/icons_svg/gp_only_selected.svg
      ../../../../release/datafiles/icons_svg/gp_select_between_strokes.svg
      ../../../../release/datafiles/icons_svg/gp_select_points.svg
      ../../../../release/datafiles/icons_svg/gp_select_strokes.svg
      ../../../../release/datafiles/icons_svg/graph.svg
      ../../../../release/datafiles/icons_svg/greasepencil.svg
      ../../../../release/datafiles/icons_svg/grid.svg
      ../../../../release/datafiles/icons_svg/grip.svg
      ../../../../release/datafiles/icons_svg/group.svg
      ../../../../release/datafiles/icons_svg/group_bone.svg
      ../../../../release/datafiles/icons_svg/group_uvs.svg
      ../../../../release/datafiles/icons_svg/group_vcol.svg
      ../../../../release/datafiles/icons_svg/group_vertex.svg
      ../../../../release/datafiles/icons_svg/hand.svg
      ../../../../release/datafiles/icons_svg/handle_aligned.svg
      ../../../../release/datafiles/icons_svg/handle_auto.svg
      ../../../../release/datafiles/icons_svg/handle_autoclamped.svg
      ../../../../release/datafiles/icons_svg/handle_free.svg
      ../../../../release/datafiles/icons_svg/handle_vector.svg
      ../../../../release/datafiles/icons_svg/heart.svg
      ../../../../release/datafiles/icons_svg/help.svg
      ../../../../release/datafiles/icons_svg/hide_off.svg
      ../../../../release/datafiles/icons_svg/hide_on.svg
      ../../../../release/datafiles/icons_svg/holdout_off.svg
      ../../../../release/datafiles/icons_svg/holdout_on.svg
      ../../../../release/datafiles/icons_svg/home.svg
      ../../../../release/datafiles/icons_svg/hook.svg
      ../../../../release/datafiles/icons_svg/image.svg
      ../../../../release/datafiles/icons_svg/image_alpha.svg
      ../../../../release/datafiles/icons_svg/image_background.svg
      ../../../../release/datafiles/icons_svg/image_data.svg
      ../../../../release/datafiles/icons_svg/image_plane.svg
      ../../../../release/datafiles/icons_svg/image_reference.svg
      ../../../../release/datafiles/icons_svg/image_rgb.svg
      ../../../../release/datafiles/icons_svg/image_rgb_alpha.svg
      ../../../../release/datafiles/icons_svg/image_zdepth.svg
      ../../../../release/datafiles/icons_svg/imgdisplay.svg
      ../../../../release/datafiles/icons_svg/import.svg
      ../../../../release/datafiles/icons_svg/indirect_only_off.svg
      ../../../../release/datafiles/icons_svg/indirect_only_on.svg
      ../../../../release/datafiles/icons_svg/info.svg
      ../../../../release/datafiles/icons_svg/internet.svg
      ../../../../release/datafiles/icons_svg/internet_offline.svg
      ../../../../release/datafiles/icons_svg/inversesquarecurve.svg
      ../../../../release/datafiles/icons_svg/ipo_back.svg
      ../../../../release/datafiles/icons_svg/ipo_bezier.svg
      ../../../../release/datafiles/icons_svg/ipo_bounce.svg
      ../../../../release/datafiles/icons_svg/ipo_circ.svg
      ../../../../release/datafiles/icons_svg/ipo_constant.svg
      ../../../../release/datafiles/icons_svg/ipo_cubic.svg
      ../../../../release/datafiles/icons_svg/ipo_ease_in.svg
      ../../../../release/datafiles/icons_svg/ipo_ease_in_out.svg
      ../../../../release/datafiles/icons_svg/ipo_ease_out.svg
      ../../../../release/datafiles/icons_svg/ipo_elastic.svg
      ../../../../release/datafiles/icons_svg/ipo_expo.svg
      ../../../../release/datafiles/icons_svg/ipo_linear.svg
      ../../../../release/datafiles/icons_svg/ipo_quad.svg
      ../../../../release/datafiles/icons_svg/ipo_quart.svg
      ../../../../release/datafiles/icons_svg/ipo_quint.svg
      ../../../../release/datafiles/icons_svg/ipo_sine.svg
      ../../../../release/datafiles/icons_svg/italic.svg
      ../../../../release/datafiles/icons_svg/keyframe.svg
      ../../../../release/datafiles/icons_svg/keyframe_hlt.svg
      ../../../../release/datafiles/icons_svg/keyingset.svg
      ../../../../release/datafiles/icons_svg/key_dehlt.svg
      ../../../../release/datafiles/icons_svg/key_hlt.svg
      ../../../../release/datafiles/icons_svg/lattice_data.svg
      ../../../../release/datafiles/icons_svg/layer_active.svg
      ../../../../release/datafiles/icons_svg/layer_used.svg
      ../../../../release/datafiles/icons_svg/library_data_broken.svg
      ../../../../release/datafiles/icons_svg/library_data_direct.svg
      ../../../../release/datafiles/icons_svg/library_data_override.svg
      ../../../../release/datafiles/icons_svg/light.svg
      ../../../../release/datafiles/icons_svg/lightprobe_plane.svg
      ../../../../release/datafiles/icons_svg/lightprobe_sphere.svg
      ../../../../release/datafiles/icons_svg/lightprobe_volume.svg
      ../../../../release/datafiles/icons_svg/light_area.svg
      ../../../../release/datafiles/icons_svg/light_data.svg
      ../../../../release/datafiles/icons_svg/light_hemi.svg
      ../../../../release/datafiles/icons_svg/light_point.svg
      ../../../../release/datafiles/icons_svg/light_spot.svg
      ../../../../release/datafiles/icons_svg/light_sun.svg
      ../../../../release/datafiles/icons_svg/lincurve.svg
      ../../../../release/datafiles/icons_svg/linenumbers_off.svg
      ../../../../release/datafiles/icons_svg/linenumbers_on.svg
      ../../../../release/datafiles/icons_svg/line_data.svg
      ../../../../release/datafiles/icons_svg/linked.svg
      ../../../../release/datafiles/icons_svg/link_blend.svg
      ../../../../release/datafiles/icons_svg/locked.svg
      ../../../../release/datafiles/icons_svg/lockview_off.svg
      ../../../../release/datafiles/icons_svg/lockview_on.svg
      ../../../../release/datafiles/icons_svg/longdisplay.svg
      ../../../../release/datafiles/icons_svg/loop_back.svg
      ../../../../release/datafiles/icons_svg/loop_forwards.svg
      ../../../../release/datafiles/icons_svg/marker.svg
      ../../../../release/datafiles/icons_svg/marker_hlt.svg
      ../../../../release/datafiles/icons_svg/matcloth.svg
      ../../../../release/datafiles/icons_svg/matcube.svg
      ../../../../release/datafiles/icons_svg/material.svg
      ../../../../release/datafiles/icons_svg/material_data.svg
      ../../../../release/datafiles/icons_svg/matfluid.svg
      ../../../../release/datafiles/icons_svg/matplane.svg
      ../../../../release/datafiles/icons_svg/matshaderball.svg
      ../../../../release/datafiles/icons_svg/matsphere.svg
      ../../../../release/datafiles/icons_svg/mat_sphere_sky.svg
      ../../../../release/datafiles/icons_svg/memory.svg
      ../../../../release/datafiles/icons_svg/menu_panel.svg
      ../../../../release/datafiles/icons_svg/mesh_capsule.svg
      ../../../../release/datafiles/icons_svg/mesh_circle.svg
      ../../../../release/datafiles/icons_svg/mesh_cone.svg
      ../../../../release/datafiles/icons_svg/mesh_cube.svg
      ../../../../release/datafiles/icons_svg/mesh_cylinder.svg
      ../../../../release/datafiles/icons_svg/mesh_data.svg
      ../../../../release/datafiles/icons_svg/mesh_grid.svg
      ../../../../release/datafiles/icons_svg/mesh_icosphere.svg
      ../../../../release/datafiles/icons_svg/mesh_monkey.svg
      ../../../../release/datafiles/icons_svg/mesh_plane.svg
      ../../../../release/datafiles/icons_svg/mesh_torus.svg
      ../../../../release/datafiles/icons_svg/mesh_uvsphere.svg
      ../../../../release/datafiles/icons_svg/meta_ball.svg
      ../../../../release/datafiles/icons_svg/meta_capsule.svg
      ../../../../release/datafiles/icons_svg/meta_cube.svg
      ../../../../release/datafiles/icons_svg/meta_data.svg
      ../../../../release/datafiles/icons_svg/meta_ellipsoid.svg
      ../../../../release/datafiles/icons_svg/meta_plane.svg
      ../../../../release/datafiles/icons_svg/modifier.svg
      ../../../../release/datafiles/icons_svg/modifier_data.svg
      ../../../../release/datafiles/icons_svg/modifier_off.svg
      ../../../../release/datafiles/icons_svg/modifier_on.svg
      ../../../../release/datafiles/icons_svg/mod_armature.svg
      ../../../../release/datafiles/icons_svg/mod_array.svg
      ../../../../release/datafiles/icons_svg/mod_bevel.svg
      ../../../../release/datafiles/icons_svg/mod_boolean.svg
      ../../../../release/datafiles/icons_svg/mod_build.svg
      ../../../../release/datafiles/icons_svg/mod_cast.svg
      ../../../../release/datafiles/icons_svg/mod_cloth.svg
      ../../../../release/datafiles/icons_svg/mod_curve.svg
      ../../../../release/datafiles/icons_svg/mod_dash.svg
      ../../../../release/datafiles/icons_svg/mod_data_transfer.svg
      ../../../../release/datafiles/icons_svg/mod_decim.svg
      ../../../../release/datafiles/icons_svg/mod_displace.svg
      ../../../../release/datafiles/icons_svg/mod_dynamicpaint.svg
      ../../../../release/datafiles/icons_svg/mod_edgesplit.svg
      ../../../../release/datafiles/icons_svg/mod_envelope.svg
      ../../../../release/datafiles/icons_svg/mod_explode.svg
      ../../../../release/datafiles/icons_svg/mod_fluid.svg
      ../../../../release/datafiles/icons_svg/mod_fluidsim.svg
      ../../../../release/datafiles/icons_svg/mod_hue_saturation.svg
      ../../../../release/datafiles/icons_svg/mod_instance.svg
      ../../../../release/datafiles/icons_svg/mod_lattice.svg
      ../../../../release/datafiles/icons_svg/mod_length.svg
      ../../../../release/datafiles/icons_svg/mod_lineart.svg
      ../../../../release/datafiles/icons_svg/mod_mask.svg
      ../../../../release/datafiles/icons_svg/mod_meshdeform.svg
      ../../../../release/datafiles/icons_svg/mod_mirror.svg
      ../../../../release/datafiles/icons_svg/mod_multires.svg
      ../../../../release/datafiles/icons_svg/mod_noise.svg
      ../../../../release/datafiles/icons_svg/mod_normaledit.svg
      ../../../../release/datafiles/icons_svg/mod_ocean.svg
      ../../../../release/datafiles/icons_svg/mod_offset.svg
      ../../../../release/datafiles/icons_svg/mod_opacity.svg
      ../../../../release/datafiles/icons_svg/mod_outline.svg
      ../../../../release/datafiles/icons_svg/mod_particles.svg
      ../../../../release/datafiles/icons_svg/mod_particle_instance.svg
      ../../../../release/datafiles/icons_svg/mod_physics.svg
      ../../../../release/datafiles/icons_svg/mod_remesh.svg
      ../../../../release/datafiles/icons_svg/mod_screw.svg
      ../../../../release/datafiles/icons_svg/mod_shrinkwrap.svg
      ../../../../release/datafiles/icons_svg/mod_simpledeform.svg
      ../../../../release/datafiles/icons_svg/mod_simplify.svg
      ../../../../release/datafiles/icons_svg/mod_skin.svg
      ../../../../release/datafiles/icons_svg/mod_smooth.svg
      ../../../../release/datafiles/icons_svg/mod_soft.svg
      ../../../../release/datafiles/icons_svg/mod_solidify.svg
      ../../../../release/datafiles/icons_svg/mod_subsurf.svg
      ../../../../release/datafiles/icons_svg/mod_thickness.svg
      ../../../../release/datafiles/icons_svg/mod_time.svg
      ../../../../release/datafiles/icons_svg/mod_tint.svg
      ../../../../release/datafiles/icons_svg/mod_triangulate.svg
      ../../../../release/datafiles/icons_svg/mod_uvproject.svg
      ../../../../release/datafiles/icons_svg/mod_vertex_weight.svg
      ../../../../release/datafiles/icons_svg/mod_warp.svg
      ../../../../release/datafiles/icons_svg/mod_wave.svg
      ../../../../release/datafiles/icons_svg/mod_wireframe.svg
      ../../../../release/datafiles/icons_svg/monkey.svg
      ../../../../release/datafiles/icons_svg/mouse_lmb.svg
      ../../../../release/datafiles/icons_svg/mouse_lmb_drag.svg
      ../../../../release/datafiles/icons_svg/mouse_mmb.svg
      ../../../../release/datafiles/icons_svg/mouse_mmb_drag.svg
      ../../../../release/datafiles/icons_svg/mouse_move.svg
      ../../../../release/datafiles/icons_svg/mouse_rmb.svg
      ../../../../release/datafiles/icons_svg/mouse_rmb_drag.svg
      ../../../../release/datafiles/icons_svg/mute_ipo_off.svg
      ../../../../release/datafiles/icons_svg/mute_ipo_on.svg
      ../../../../release/datafiles/icons_svg/network_drive.svg
      ../../../../release/datafiles/icons_svg/newfolder.svg
      ../../../../release/datafiles/icons_svg/next_keyframe.svg
      ../../../../release/datafiles/icons_svg/nla.svg
      ../../../../release/datafiles/icons_svg/nla_pushdown.svg
      ../../../../release/datafiles/icons_svg/nocurve.svg
      ../../../../release/datafiles/icons_svg/node.svg
      ../../../../release/datafiles/icons_svg/nodetree.svg
      ../../../../release/datafiles/icons_svg/node_compositing.svg
      ../../../../release/datafiles/icons_svg/node_corner.svg
      ../../../../release/datafiles/icons_svg/node_insert_off.svg
      ../../../../release/datafiles/icons_svg/node_insert_on.svg
      ../../../../release/datafiles/icons_svg/node_material.svg
      ../../../../release/datafiles/icons_svg/node_sel.svg
      ../../../../release/datafiles/icons_svg/node_side.svg
      ../../../../release/datafiles/icons_svg/node_texture.svg
      ../../../../release/datafiles/icons_svg/node_top.svg
      ../../../../release/datafiles/icons_svg/none.svg
      ../../../../release/datafiles/icons_svg/normalize_fcurves.svg
      ../../../../release/datafiles/icons_svg/normals_face.svg
      ../../../../release/datafiles/icons_svg/normals_vertex.svg
      ../../../../release/datafiles/icons_svg/normals_vertex_face.svg
      ../../../../release/datafiles/icons_svg/object_data.svg
      ../../../../release/datafiles/icons_svg/object_datamode.svg
      ../../../../release/datafiles/icons_svg/object_hidden.svg
      ../../../../release/datafiles/icons_svg/object_origin.svg
      ../../../../release/datafiles/icons_svg/onionskin_off.svg
      ../../../../release/datafiles/icons_svg/onionskin_on.svg
      ../../../../release/datafiles/icons_svg/options.svg
      ../../../../release/datafiles/icons_svg/orientation_cursor.svg
      ../../../../release/datafiles/icons_svg/orientation_gimbal.svg
      ../../../../release/datafiles/icons_svg/orientation_global.svg
      ../../../../release/datafiles/icons_svg/orientation_local.svg
      ../../../../release/datafiles/icons_svg/orientation_normal.svg
      ../../../../release/datafiles/icons_svg/orientation_parent.svg
      ../../../../release/datafiles/icons_svg/orientation_view.svg
      ../../../../release/datafiles/icons_svg/orphan_data.svg
      ../../../../release/datafiles/icons_svg/outliner.svg
      ../../../../release/datafiles/icons_svg/outliner_collection.svg
      ../../../../release/datafiles/icons_svg/outliner_data_armature.svg
      ../../../../release/datafiles/icons_svg/outliner_data_camera.svg
      ../../../../release/datafiles/icons_svg/outliner_data_curve.svg
      ../../../../release/datafiles/icons_svg/outliner_data_curves.svg
      ../../../../release/datafiles/icons_svg/outliner_data_empty.svg
      ../../../../release/datafiles/icons_svg/outliner_data_font.svg
      ../../../../release/datafiles/icons_svg/outliner_data_gp_layer.svg
      ../../../../release/datafiles/icons_svg/outliner_data_greasepencil.svg
      ../../../../release/datafiles/icons_svg/outliner_data_lattice.svg
      ../../../../release/datafiles/icons_svg/outliner_data_light.svg
      ../../../../release/datafiles/icons_svg/outliner_data_lightprobe.svg
      ../../../../release/datafiles/icons_svg/outliner_data_mesh.svg
      ../../../../release/datafiles/icons_svg/outliner_data_meta.svg
      ../../../../release/datafiles/icons_svg/outliner_data_pointcloud.svg
      ../../../../release/datafiles/icons_svg/outliner_data_speaker.svg
      ../../../../release/datafiles/icons_svg/outliner_data_surface.svg
      ../../../../release/datafiles/icons_svg/outliner_data_volume.svg
      ../../../../release/datafiles/icons_svg/outliner_ob_armature.svg
      ../../../../release/datafiles/icons_svg/outliner_ob_camera.svg
      ../../../../release/datafiles/icons_svg/outliner_ob_curve.svg
      ../../../../release/datafiles/icons_svg/outliner_ob_curves.svg
      ../../../../release/datafiles/icons_svg/outliner_ob_empty.svg
      ../../../../release/datafiles/icons_svg/outliner_ob_font.svg
      ../../../../release/datafiles/icons_svg/outliner_ob_force_field.svg
      ../../../../release/datafiles/icons_svg/outliner_ob_greasepencil.svg
      ../../../../release/datafiles/icons_svg/outliner_ob_group_instance.svg
      ../../../../release/datafiles/icons_svg/outliner_ob_image.svg
      ../../../../release/datafiles/icons_svg/outliner_ob_lattice.svg
      ../../../../release/datafiles/icons_svg/outliner_ob_light.svg
      ../../../../release/datafiles/icons_svg/outliner_ob_lightprobe.svg
      ../../../../release/datafiles/icons_svg/outliner_ob_mesh.svg
      ../../../../release/datafiles/icons_svg/outliner_ob_meta.svg
      ../../../../release/datafiles/icons_svg/outliner_ob_pointcloud.svg
      ../../../../release/datafiles/icons_svg/outliner_ob_speaker.svg
      ../../../../release/datafiles/icons_svg/outliner_ob_surface.svg
      ../../../../release/datafiles/icons_svg/outliner_ob_volume.svg
      ../../../../release/datafiles/icons_svg/output.svg
      ../../../../release/datafiles/icons_svg/overlay.svg
      ../../../../release/datafiles/icons_svg/package.svg
      ../../../../release/datafiles/icons_svg/panel_close.svg
      ../../../../release/datafiles/icons_svg/particlemode.svg
      ../../../../release/datafiles/icons_svg/particles.svg
      ../../../../release/datafiles/icons_svg/particle_data.svg
      ../../../../release/datafiles/icons_svg/particle_path.svg
      ../../../../release/datafiles/icons_svg/particle_point.svg
      ../../../../release/datafiles/icons_svg/particle_tip.svg
      ../../../../release/datafiles/icons_svg/pastedown.svg
      ../../../../release/datafiles/icons_svg/pasteflipdown.svg
      ../../../../release/datafiles/icons_svg/pasteflipup.svg
      ../../../../release/datafiles/icons_svg/pause.svg
      ../../../../release/datafiles/icons_svg/physics.svg
      ../../../../release/datafiles/icons_svg/pinned.svg
      ../../../../release/datafiles/icons_svg/pivot_active.svg
      ../../../../release/datafiles/icons_svg/pivot_boundbox.svg
      ../../../../release/datafiles/icons_svg/pivot_cursor.svg
      ../../../../release/datafiles/icons_svg/pivot_individual.svg
      ../../../../release/datafiles/icons_svg/pivot_median.svg
      ../../../../release/datafiles/icons_svg/play.svg
      ../../../../release/datafiles/icons_svg/play_reverse.svg
      ../../../../release/datafiles/icons_svg/play_sound.svg
      ../../../../release/datafiles/icons_svg/plugin.svg
      ../../../../release/datafiles/icons_svg/plus.svg
      ../../../../release/datafiles/icons_svg/pmarker.svg
      ../../../../release/datafiles/icons_svg/pmarker_act.svg
      ../../../../release/datafiles/icons_svg/pmarker_sel.svg
      ../../../../release/datafiles/icons_svg/pointcloud_data.svg
      ../../../../release/datafiles/icons_svg/pointcloud_point.svg
      ../../../../release/datafiles/icons_svg/pose_hlt.svg
      ../../../../release/datafiles/icons_svg/preferences.svg
      ../../../../release/datafiles/icons_svg/preset.svg
      ../../../../release/datafiles/icons_svg/preset_new.svg
      ../../../../release/datafiles/icons_svg/preview_range.svg
      ../../../../release/datafiles/icons_svg/prev_keyframe.svg
      ../../../../release/datafiles/icons_svg/properties.svg
      ../../../../release/datafiles/icons_svg/prop_con.svg
      ../../../../release/datafiles/icons_svg/prop_off.svg
      ../../../../release/datafiles/icons_svg/prop_on.svg
      ../../../../release/datafiles/icons_svg/prop_projected.svg
      ../../../../release/datafiles/icons_svg/question.svg
      ../../../../release/datafiles/icons_svg/quit.svg
      ../../../../release/datafiles/icons_svg/radiobut_off.svg
      ../../../../release/datafiles/icons_svg/radiobut_on.svg
      ../../../../release/datafiles/icons_svg/rec.svg
      ../../../../release/datafiles/icons_svg/record_off.svg
      ../../../../release/datafiles/icons_svg/record_on.svg
      ../../../../release/datafiles/icons_svg/recover_last.svg
      ../../../../release/datafiles/icons_svg/remove.svg
      ../../../../release/datafiles/icons_svg/renderlayers.svg
      ../../../../release/datafiles/icons_svg/render_animation.svg
      ../../../../release/datafiles/icons_svg/render_result.svg
      ../../../../release/datafiles/icons_svg/render_still.svg
      ../../../../release/datafiles/icons_svg/restrict_color_off.svg
      ../../../../release/datafiles/icons_svg/restrict_color_on.svg
      ../../../../release/datafiles/icons_svg/restrict_instanced_off.svg
      ../../../../release/datafiles/icons_svg/restrict_instanced_on.svg
      ../../../../release/datafiles/icons_svg/restrict_render_off.svg
      ../../../../release/datafiles/icons_svg/restrict_render_on.svg
      ../../../../release/datafiles/icons_svg/restrict_select_off.svg
      ../../../../release/datafiles/icons_svg/restrict_select_on.svg
      ../../../../release/datafiles/icons_svg/restrict_view_off.svg
      ../../../../release/datafiles/icons_svg/restrict_view_on.svg
      ../../../../release/datafiles/icons_svg/rew.svg
      ../../../../release/datafiles/icons_svg/rightarrow.svg
      ../../../../release/datafiles/icons_svg/rightarrow_thin.svg
      ../../../../release/datafiles/icons_svg/rigid_body.svg
      ../../../../release/datafiles/icons_svg/rigid_body_constraint.svg
      ../../../../release/datafiles/icons_svg/rna.svg
      ../../../../release/datafiles/icons_svg/rna_add.svg
      ../../../../release/datafiles/icons_svg/rndcurve.svg
      ../../../../release/datafiles/icons_svg/rootcurve.svg
      ../../../../release/datafiles/icons_svg/scene.svg
      ../../../../release/datafiles/icons_svg/scene_data.svg
      ../../../../release/datafiles/icons_svg/screen_back.svg
      ../../../../release/datafiles/icons_svg/script.svg
      ../../../../release/datafiles/icons_svg/scriptplugins.svg
      ../../../../release/datafiles/icons_svg/sculptmode_hlt.svg
      ../../../../release/datafiles/icons_svg/select_difference.svg
      ../../../../release/datafiles/icons_svg/select_extend.svg
      ../../../../release/datafiles/icons_svg/select_intersect.svg
      ../../../../release/datafiles/icons_svg/select_set.svg
      ../../../../release/datafiles/icons_svg/select_subtract.svg
      ../../../../release/datafiles/icons_svg/sequence.svg
      ../../../../release/datafiles/icons_svg/seq_chroma_scope.svg
      ../../../../release/datafiles/icons_svg/seq_histogram.svg
      ../../../../release/datafiles/icons_svg/seq_luma_waveform.svg
      ../../../../release/datafiles/icons_svg/seq_preview.svg
      ../../../../release/datafiles/icons_svg/seq_sequencer.svg
      ../../../../release/datafiles/icons_svg/seq_splitview.svg
      ../../../../release/datafiles/icons_svg/seq_strip_duplicate.svg
      ../../../../release/datafiles/icons_svg/seq_strip_meta.svg
      ../../../../release/datafiles/icons_svg/settings.svg
      ../../../../release/datafiles/icons_svg/shaderfx.svg
      ../../../../release/datafiles/icons_svg/shading_bbox.svg
      ../../../../release/datafiles/icons_svg/shading_rendered.svg
      ../../../../release/datafiles/icons_svg/shading_solid.svg
      ../../../../release/datafiles/icons_svg/shading_texture.svg
      ../../../../release/datafiles/icons_svg/shading_wire.svg
      ../../../../release/datafiles/icons_svg/shapekey_data.svg
      ../../../../release/datafiles/icons_svg/sharpcurve.svg
      ../../../../release/datafiles/icons_svg/shortdisplay.svg
      ../../../../release/datafiles/icons_svg/small_caps.svg
      ../../../../release/datafiles/icons_svg/smoothcurve.svg
      ../../../../release/datafiles/icons_svg/snap_edge.svg
      ../../../../release/datafiles/icons_svg/snap_face.svg
      ../../../../release/datafiles/icons_svg/snap_face_center.svg
      ../../../../release/datafiles/icons_svg/snap_face_nearest.svg
      ../../../../release/datafiles/icons_svg/snap_grid.svg
      ../../../../release/datafiles/icons_svg/snap_increment.svg
      ../../../../release/datafiles/icons_svg/snap_midpoint.svg
      ../../../../release/datafiles/icons_svg/snap_normal.svg
      ../../../../release/datafiles/icons_svg/snap_off.svg
      ../../../../release/datafiles/icons_svg/snap_on.svg
      ../../../../release/datafiles/icons_svg/snap_peel_object.svg
      ../../../../release/datafiles/icons_svg/snap_perpendicular.svg
      ../../../../release/datafiles/icons_svg/snap_vertex.svg
      ../../../../release/datafiles/icons_svg/snap_volume.svg
      ../../../../release/datafiles/icons_svg/solo_off.svg
      ../../../../release/datafiles/icons_svg/solo_on.svg
      ../../../../release/datafiles/icons_svg/sortalpha.svg
      ../../../../release/datafiles/icons_svg/sortbyext.svg
      ../../../../release/datafiles/icons_svg/sortsize.svg
      ../../../../release/datafiles/icons_svg/sorttime.svg
      ../../../../release/datafiles/icons_svg/sort_asc.svg
      ../../../../release/datafiles/icons_svg/sort_desc.svg
      ../../../../release/datafiles/icons_svg/sound.svg
      ../../../../release/datafiles/icons_svg/speaker.svg
      ../../../../release/datafiles/icons_svg/sphere.svg
      ../../../../release/datafiles/icons_svg/spherecurve.svg
      ../../../../release/datafiles/icons_svg/split_horizontal.svg
      ../../../../release/datafiles/icons_svg/split_vertical.svg
      ../../../../release/datafiles/icons_svg/spreadsheet.svg
      ../../../../release/datafiles/icons_svg/statusbar.svg
      ../../../../release/datafiles/icons_svg/sticky_uvs_disable.svg
      ../../../../release/datafiles/icons_svg/sticky_uvs_loc.svg
      ../../../../release/datafiles/icons_svg/sticky_uvs_vert.svg
      ../../../../release/datafiles/icons_svg/strands.svg
      ../../../../release/datafiles/icons_svg/stroke.svg
      ../../../../release/datafiles/icons_svg/stylus_pressure.svg
      ../../../../release/datafiles/icons_svg/surface_data.svg
      ../../../../release/datafiles/icons_svg/surface_ncircle.svg
      ../../../../release/datafiles/icons_svg/surface_ncurve.svg
      ../../../../release/datafiles/icons_svg/surface_ncylinder.svg
      ../../../../release/datafiles/icons_svg/surface_nsphere.svg
      ../../../../release/datafiles/icons_svg/surface_nsurface.svg
      ../../../../release/datafiles/icons_svg/surface_ntorus.svg
      ../../../../release/datafiles/icons_svg/syntax_off.svg
      ../../../../release/datafiles/icons_svg/syntax_on.svg
      ../../../../release/datafiles/icons_svg/system.svg
      ../../../../release/datafiles/icons_svg/tag.svg
      ../../../../release/datafiles/icons_svg/temp.svg
      ../../../../release/datafiles/icons_svg/text.svg
      ../../../../release/datafiles/icons_svg/texture.svg
      ../../../../release/datafiles/icons_svg/texture_data.svg
      ../../../../release/datafiles/icons_svg/three_dots.svg
      ../../../../release/datafiles/icons_svg/time.svg
      ../../../../release/datafiles/icons_svg/tool_settings.svg
      ../../../../release/datafiles/icons_svg/topbar.svg
      ../../../../release/datafiles/icons_svg/tpaint_hlt.svg
      ../../../../release/datafiles/icons_svg/tracker.svg
      ../../../../release/datafiles/icons_svg/tracker_data.svg
      ../../../../release/datafiles/icons_svg/tracking.svg
      ../../../../release/datafiles/icons_svg/tracking_backwards.svg
      ../../../../release/datafiles/icons_svg/tracking_backwards_single.svg
      ../../../../release/datafiles/icons_svg/tracking_clear_backwards.svg
      ../../../../release/datafiles/icons_svg/tracking_clear_forwards.svg
      ../../../../release/datafiles/icons_svg/tracking_forwards.svg
      ../../../../release/datafiles/icons_svg/tracking_forwards_single.svg
      ../../../../release/datafiles/icons_svg/tracking_refine_backwards.svg
      ../../../../release/datafiles/icons_svg/tracking_refine_forwards.svg
      ../../../../release/datafiles/icons_svg/transform_origins.svg
      ../../../../release/datafiles/icons_svg/trash.svg
      ../../../../release/datafiles/icons_svg/tria_down.svg
      ../../../../release/datafiles/icons_svg/tria_down_bar.svg
      ../../../../release/datafiles/icons_svg/tria_left.svg
      ../../../../release/datafiles/icons_svg/tria_left_bar.svg
      ../../../../release/datafiles/icons_svg/tria_right.svg
      ../../../../release/datafiles/icons_svg/tria_right_bar.svg
      ../../../../release/datafiles/icons_svg/tria_up.svg
      ../../../../release/datafiles/icons_svg/tria_up_bar.svg
      ../../../../release/datafiles/icons_svg/uglypackage.svg
      ../../../../release/datafiles/icons_svg/underline.svg
      ../../../../release/datafiles/icons_svg/unlinked.svg
      ../../../../release/datafiles/icons_svg/unlocked.svg
      ../../../../release/datafiles/icons_svg/unpinned.svg
      ../../../../release/datafiles/icons_svg/url.svg
      ../../../../release/datafiles/icons_svg/user.svg
      ../../../../release/datafiles/icons_svg/uv.svg
      ../../../../release/datafiles/icons_svg/uv_data.svg
      ../../../../release/datafiles/icons_svg/uv_edgesel.svg
      ../../../../release/datafiles/icons_svg/uv_facesel.svg
      ../../../../release/datafiles/icons_svg/uv_islandsel.svg
      ../../../../release/datafiles/icons_svg/uv_sync_select.svg
      ../../../../release/datafiles/icons_svg/uv_vertexsel.svg
      ../../../../release/datafiles/icons_svg/vertexsel.svg
      ../../../../release/datafiles/icons_svg/view3d.svg
      ../../../../release/datafiles/icons_svg/viewzoom.svg
      ../../../../release/datafiles/icons_svg/view_camera.svg
      ../../../../release/datafiles/icons_svg/view_camera_unselected.svg
      ../../../../release/datafiles/icons_svg/view_locked.svg
      ../../../../release/datafiles/icons_svg/view_ortho.svg
      ../../../../release/datafiles/icons_svg/view_pan.svg
      ../../../../release/datafiles/icons_svg/view_perspective.svg
      ../../../../release/datafiles/icons_svg/view_unlocked.svg
      ../../../../release/datafiles/icons_svg/view_zoom.svg
      ../../../../release/datafiles/icons_svg/vis_sel_00.svg
      ../../../../release/datafiles/icons_svg/vis_sel_01.svg
      ../../../../release/datafiles/icons_svg/vis_sel_10.svg
      ../../../../release/datafiles/icons_svg/vis_sel_11.svg
      ../../../../release/datafiles/icons_svg/volume_data.svg
      ../../../../release/datafiles/icons_svg/vpaint_hlt.svg
      ../../../../release/datafiles/icons_svg/window.svg
      ../../../../release/datafiles/icons_svg/wordwrap_off.svg
      ../../../../release/datafiles/icons_svg/wordwrap_on.svg
      ../../../../release/datafiles/icons_svg/workspace.svg
      ../../../../release/datafiles/icons_svg/world.svg
      ../../../../release/datafiles/icons_svg/world_data.svg
      ../../../../release/datafiles/icons_svg/wpaint_hlt.svg
      ../../../../release/datafiles/icons_svg/x.svg
      ../../../../release/datafiles/icons_svg/xray.svg
      ../../../../release/datafiles/icons_svg/zoom_all.svg
      ../../../../release/datafiles/icons_svg/zoom_in.svg
      ../../../../release/datafiles/icons_svg/zoom_out.svg
      ../../../../release/datafiles/icons_svg/zoom_previous.svg
      ../../../../release/datafiles/icons_svg/zoom_selected.svg
    )
    set(SVGHCONTENTS )
    set(SVGCCCONTENTS )
    foreach(svg ${SVG_SRC})
      data_to_c_simple(${svg} SRC)
      get_filename_component(svg_name ${svg} NAME)
        string(REPLACE ".svg" "" svg_name ${svg_name})
        string(REPLACE "." "_" svg_name ${svg_name})
        string(TOUPPER ${svg_name} svg_name_upper)
      set(SVGHCONTENTS "${SVGHCONTENTS}extern const char datatoc_${svg_name}_svg[];\n")
      set(SVGCCCONTENTS "${SVGCCCONTENTS}case ICON_${svg_name_upper}: return datatoc_${svg_name}_svg;\n")
    endforeach()
    configure_file(
      "svg_icons.h.in"
      "${CMAKE_CURRENT_BINARY_DIR}/svg_icons.h"
      ESCAPE_QUOTES
      @ONLY
    )
    configure_file(
      "svg_icons.cc.in"
      "${CMAKE_CURRENT_BINARY_DIR}/svg_icons.cc"
      ESCAPE_QUOTES
      @ONLY
    )
    list(APPEND SRC ${CMAKE_CURRENT_BINARY_DIR}/svg_icons.h)
    list(APPEND SRC ${CMAKE_CURRENT_BINARY_DIR}/svg_icons.cc)
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
add_library(bf::editor::datafiles ALIAS bf_editor_datafiles)
