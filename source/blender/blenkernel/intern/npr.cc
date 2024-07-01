/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 */

#include "BKE_node.hh"
#include "DNA_node_types.h"
#include "NOD_shader.h"

bNodeTree *BKE_npr_tree_add(Main *bmain, const char *name)
{
  bNodeTree *ntree = blender::bke::ntreeAddTree(bmain, name, ntreeType_Shader->idname);
  return ntree;
}
