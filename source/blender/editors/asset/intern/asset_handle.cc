/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edasset
 */

#include <string>

#include "BKE_icons.h"

#include "DNA_space_types.h"

#include "DNA_space_types.h"

#include "ED_fileselect.hh"

#include "ED_asset_handle.hh"

namespace blender::ed::asset {

asset_system::AssetRepresentation *handle_get_representation(const AssetHandle *asset)
{
  return asset->file_data->asset;
}

int ED_asset_handle_get_preview_icon_id(const AssetHandle *asset_handle)
{
  PreviewImage *preview = AS_asset_representation_preview_request(asset_handle->file_data->asset);
  /* Can be null. */
  ID *local_id = AS_asset_representation_local_id_get(asset_handle->file_data->asset);

  return BKE_icon_preview_ensure(local_id, preview);
}

int handle_get_preview_or_type_icon_id(const AssetHandle *asset)
{
  return ED_file_icon(asset->file_data);
}

}  // namespace blender::ed::asset
