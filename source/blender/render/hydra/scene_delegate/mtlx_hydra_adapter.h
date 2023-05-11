/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once

#include <pxr/base/tf/token.h>
#include <pxr/imaging/hd/material.h>

namespace blender::render::hydra {

void hdmtlx_convert_to_materialnetworkmap(std::string const &mtlx_path,
                                          pxr::TfTokenVector const &shader_source_types,
                                          pxr::TfTokenVector const &render_contexts,
                                          pxr::HdMaterialNetworkMap *out);

}  // namespace blender::render::hydra
