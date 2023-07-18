# SPDX-FileCopyrightText: 2023 Blender Foundation
#
# SPDX-License-Identifier: GPL-2.0-or-later

set(INC
  ../include
  ../../blenloader
  ../../blentranslation
  ../../depsgraph
  ../../gpu
  ../../windowmanager

)

set(INC_SYS
)

set(SRC
  nla_buttons.cc
  nla_channels.cc
  nla_draw.cc
  nla_edit.cc
  nla_ops.cc
  nla_select.cc
  space_nla.cc

  nla_intern.hh
)

set(LIB
  PRIVATE bf::blenkernel
  PRIVATE bf::blenlib
  PRIVATE bf::dna
  PRIVATE bf::intern::guardedalloc
  PRIVATE bf::rna
)


blender_add_lib(bf_editor_space_nla "${SRC}" "${INC}" "${INC_SYS}" "${LIB}")

