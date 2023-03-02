/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once

#include <pxr/base/tf/token.h>
#include <pxr/pxr.h>

#include <string>

PXR_NAMESPACE_OPEN_SCOPE

struct HdMaterialNetworkMap;

void HdMtlxConvertToMaterialNetworkMap(std::string const &mtlxPath,
                                       TfTokenVector const &shaderSourceTypes,
                                       TfTokenVector const &renderContexts,
                                       HdMaterialNetworkMap *out);

PXR_NAMESPACE_CLOSE_SCOPE
