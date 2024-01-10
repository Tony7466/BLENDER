/* SPDX-FileCopyrightText: 2011 by Bastien Montagne. All rights reserved.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup modifiers
 */

#pragma once

#include "BLI_index_mask.hh"
#include "BLI_vector.hh"

#include "BKE_modifier.hh"

struct ARegionType;
struct GreasePencil;
struct GreasePencilModifierFilterData;
struct PanelType;
namespace blender::bke::greasepencil {
class Drawing;
}

namespace blender::greasepencil {

void init_data_filter(GreasePencilModifierFilterData *filter_data);
void copy_data_filter(const GreasePencilModifierFilterData *filter_data_src,
                      GreasePencilModifierFilterData *filter_data_dst,
                      int flag);
void free_data_filter(GreasePencilModifierFilterData *filter_data);
void foreach_ID_link_filter(GreasePencilModifierFilterData *filter_data,
                            Object *ob,
                            IDWalkFunc walk,
                            void *user_data);
void filter_subpanel_register(ARegionType *region_type, PanelType *panel_type);

IndexMask get_filtered_layer_mask(const GreasePencil &grease_pencil,
                                  const GreasePencilModifierFilterData &filter_data,
                                  IndexMaskMemory &memory);

Vector<bke::greasepencil::Drawing *> get_drawings_for_write(GreasePencil &grease_pencil,
                                                            const IndexMask &layer_mask,
                                                            int frame);

}  // namespace blender::greasepencil
