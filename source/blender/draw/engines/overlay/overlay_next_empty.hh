/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 */

#pragma once

#include "overlay_next_extra_pass.hh"
namespace blender::draw::overlay {

static void empty_sync(const ObjectRef &ob_ref,
                       const select::ID select_id,
                       Resources & /*res*/,
                       const State & /*state*/,
                       ExtraInstancePass &pass,
                       ExtraInstanceData data)
{
  pass.empty_buf(ob_ref.object->empty_drawtype).append(data, select_id);
  if (ob_ref.object->empty_drawtype) {
    /** TODO: This only shows the frame.
     * See OVERLAY_image_empty_cache_populate() for the image. */
  }
}

}  // namespace blender::draw::overlay
