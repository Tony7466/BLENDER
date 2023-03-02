/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include "mtlxHydraAdapter.h"

#include <pxr/base/arch/fileSystem.h>

#include <pxr/usd/ar/resolver.h>
#include <pxr/usd/ar/resolverContextBinder.h>
#include <pxr/usd/ar/resolverScopedCache.h>

#include <pxr/usd/usdMtlx/reader.h>
#include <pxr/usd/usdMtlx/utils.h>

#include <pxr/usd/usdShade/material.h>
#include <pxr/usd/usdShade/shader.h>

#include <pxr/usdImaging/usdImaging/materialParamUtils.h>

#include <pxr/imaging/hd/material.h>
#include <pxr/imaging/hd/tokens.h>

namespace mx = MaterialX;

PXR_NAMESPACE_OPEN_SCOPE

void HdMtlxConvertToMaterialNetworkMap(std::string const &mtlxPath,
                                       TfTokenVector const &shaderSourceTypes,
                                       TfTokenVector const &renderContexts,
                                       HdMaterialNetworkMap *out)
{
  if (mtlxPath.empty()) {
    return;
  }

  std::string basePath = TfGetPathName(mtlxPath);

  ArResolver &resolver = ArGetResolver();
  const ArResolverContext context = resolver.CreateDefaultContextForAsset(mtlxPath);
  ArResolverContextBinder binder(context);
  ArResolverScopedCache resolverCache;

  std::string mtlxName = TfGetBaseName(mtlxPath);
  std::string stageId = TfStringPrintf(
      "%s%s%s.usda", basePath.c_str(), ARCH_PATH_SEP, mtlxName.c_str());
  UsdStageRefPtr stage = UsdStage::CreateInMemory(stageId, context);

  try {
    mx::DocumentPtr doc = UsdMtlxReadDocument(mtlxPath);
    UsdMtlxRead(doc, stage);
  }
  catch (mx::ExceptionFoundCycle &x) {
    TF_RUNTIME_ERROR("MaterialX cycle found: %s\n", x.what());
    return;
  }
  catch (mx::Exception &x) {
    TF_RUNTIME_ERROR("MaterialX error: %s\n", x.what());
    return;
  }
  
  if (UsdPrim materials = stage->GetPrimAtPath(SdfPath("/MaterialX/Materials"))) {
    if (UsdPrimSiblingRange children = materials.GetChildren()) {
      if (auto material = UsdShadeMaterial(*children.begin())) {
        if (UsdShadeShader mtlxSurface = material.ComputeSurfaceSource(renderContexts)) {
          UsdImagingBuildHdMaterialNetworkFromTerminal(mtlxSurface.GetPrim(),
                                                       HdMaterialTerminalTokens->surface,
                                                       shaderSourceTypes,
                                                       renderContexts,
                                                       out,
                                                       UsdTimeCode::Default());
        }
      }
    }
  }
}

PXR_NAMESPACE_CLOSE_SCOPE
