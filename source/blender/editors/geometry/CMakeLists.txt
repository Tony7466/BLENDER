# SPDX-FileCopyrightText: 2023 Blender Foundation
#
# SPDX-License-Identifier: GPL-2.0-or-later

set(INC
  ../include
  ../../asset_system
  ../../blentranslation
  ../../depsgraph
  ../../functions
  ../../nodes
  ../../windowmanager
  ../../bmesh

)

set(INC_SYS
)

set(SRC
  geometry_attributes.cc
  geometry_ops.cc
  node_group_operator.cc

  geometry_intern.hh
)

set(LIB
  PRIVATE bf::blenkernel
  PRIVATE bf::blenlib
  bf_depsgraph
  PRIVATE bf::dna
  bf_editor_object
  PRIVATE bf::intern::guardedalloc
  PRIVATE bf::rna
  bf_windowmanager
)

blender_add_lib(bf_editor_geometry "${SRC}" "${INC}" "${INC_SYS}" "${LIB}")

