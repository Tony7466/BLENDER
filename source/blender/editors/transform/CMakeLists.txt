# SPDX-FileCopyrightText: 2023 Blender Foundation
#
# SPDX-License-Identifier: GPL-2.0-or-later

set(INC
  ../include
  ../../blenfont
  ../../blentranslation
  ../../bmesh
  ../../depsgraph
  ../../gpu
  ../../ikplugin
  ../../makesrna
  ../../render
  ../../sequencer
  ../../windowmanager
  # RNA_prototypes.h
  ${CMAKE_BINARY_DIR}/source/blender/makesrna
)
set(INC_SYS
)

set(SRC
  transform.c
  transform_constraints.cc
  transform_convert.cc
  transform_convert_action.cc
  transform_convert_armature.cc
  transform_convert_cursor.cc
  transform_convert_curve.cc
  transform_convert_curves.cc
  transform_convert_gpencil_legacy.cc
  transform_convert_graph.cc
  transform_convert_lattice.cc
  transform_convert_mask.cc
  transform_convert_mball.cc
  transform_convert_mesh.cc
  transform_convert_mesh_edge.cc
  transform_convert_mesh_skin.cc
  transform_convert_mesh_uv.cc
  transform_convert_mesh_vert_cdata.cc
  transform_convert_nla.cc
  transform_convert_node.cc
  transform_convert_object.cc
  transform_convert_object_texspace.cc
  transform_convert_paintcurve.cc
  transform_convert_particle.cc
  transform_convert_sculpt.cc
  transform_convert_sequencer.cc
  transform_convert_sequencer_image.cc
  transform_convert_tracking.cc
  transform_convert_tracking_curves.cc
  transform_draw_cursors.cc
  transform_generics.cc
  transform_gizmo_2d.cc
  transform_gizmo_3d.cc
  transform_gizmo_3d_cage.cc
  transform_gizmo_3d_shear.cc
  transform_gizmo_extrude_3d.c
  transform_input.cc
  transform_mode.c
  transform_mode_align.cc
  transform_mode_baketime.cc
  transform_mode_bbone_resize.cc
  transform_mode_bend.cc
  transform_mode_boneenvelope.cc
  transform_mode_boneroll.cc
  transform_mode_curveshrinkfatten.cc
  transform_mode_customdata.cc
  transform_mode_edge_rotate_normal.cc
  transform_mode_edge_seq_slide.cc
  transform_mode_edge_slide.c
  transform_mode_gpopacity.c
  transform_mode_gpshrinkfatten.c
  transform_mode_maskshrinkfatten.c
  transform_mode_mirror.c
  transform_mode_push_pull.c
  transform_mode_resize.c
  transform_mode_rotate.c
  transform_mode_shear.c
  transform_mode_shrink_fatten.c
  transform_mode_skin_resize.c
  transform_mode_snapsource.c
  transform_mode_tilt.c
  transform_mode_timescale.c
  transform_mode_timeslide.c
  transform_mode_timetranslate.c
  transform_mode_tosphere.c
  transform_mode_trackball.c
  transform_mode_translate.c
  transform_mode_vert_slide.c
  transform_ops.c
  transform_orientations.c
  transform_snap.cc
  transform_snap_animation.c
  transform_snap_object.cc
  transform_snap_object_armature.cc
  transform_snap_object_camera.cc
  transform_snap_object_curve.cc
  transform_snap_object_editmesh.cc
  transform_snap_object_mesh.cc
  transform_snap_sequencer.c

  transform.h
  transform_constraints.h
  transform_convert.h
  transform_data.h
  transform_draw_cursors.h
  transform_gizmo.h
  transform_mode.h
  transform_orientations.h
  transform_snap.h
  transform_snap_object.hh
)

set(LIB
  bf_blenfont
  PRIVATE bf::blenkernel
  PRIVATE bf::blenlib
  bf_bmesh
  PRIVATE bf::dna
  bf_editor_mask
  bf_gpu
  PRIVATE bf::intern::guardedalloc
)


blender_add_lib(bf_editor_transform "${SRC}" "${INC}" "${INC_SYS}" "${LIB}")

# RNA_prototypes.h
add_dependencies(bf_editor_transform bf_rna)
