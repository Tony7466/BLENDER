/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bke
 * \brief Utility functions for vertex groups in grease pencil objects
 */

#include "DNA_grease_pencil_types.h"

namespace blender::bke::greasepencil {

/** Make sure drawings only contain vertex groups of the #GreasePencil. */
void validate_drawing_vertex_groups(GreasePencil &grease_pencil);

/** Assign selected vertices to the vertex group. */
void assign_to_vertex_group(GreasePencil &grease_pencil, StringRef name, float weight);

}