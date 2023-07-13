/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include "BLI_utildefines.h"

#include <pxr/base/arch/fileSystem.h>

#include <pxr/usd/ar/resolver.h>
#include <pxr/usd/ar/resolverContextBinder.h>
#include <pxr/usd/ar/resolverScopedCache.h>

#ifdef WITH_MATERIALX
#  include <pxr/usd/usdMtlx/reader.h>
#  include <pxr/usd/usdMtlx/utils.h>
#endif

#include <pxr/usd/usdShade/material.h>
#include <pxr/usd/usdShade/shader.h>

#include <pxr/usdImaging/usdImaging/materialParamUtils.h>

#include <pxr/imaging/hd/material.h>
#include <pxr/imaging/hd/tokens.h>

#include "mtlx_hydra_adapter.h"

namespace blender::render::hydra {

void hdmtlx_convert_to_materialnetworkmap(std::string const &mtlx_path,
                                          pxr::TfTokenVector const &shader_source_types,
                                          pxr::TfTokenVector const &render_contexts,
                                          pxr::HdMaterialNetworkMap *out)
{
#ifdef WITH_MATERIALX
  if (mtlx_path.empty()) {
    return;
  }

  std::string basePath = pxr::TfGetPathName(mtlx_path);

  pxr::ArResolver &resolver = pxr::ArGetResolver();
  const pxr::ArResolverContext context = resolver.CreateDefaultContextForAsset(mtlx_path);
  pxr::ArResolverContextBinder binder(context);
  pxr::ArResolverScopedCache resolver_cache;

  std::string mtlxName = pxr::TfGetBaseName(mtlx_path);
  std::string stage_id = pxr::TfStringPrintf(
      "%s%s%s.usda", basePath.c_str(), ARCH_PATH_SEP, mtlxName.c_str());
  pxr::UsdStageRefPtr stage = pxr::UsdStage::CreateInMemory(stage_id, context);

  try {
    MaterialX::DocumentPtr doc = pxr::UsdMtlxReadDocument(mtlx_path);
    pxr::UsdMtlxRead(doc, stage);
  }
  catch (MaterialX::ExceptionFoundCycle &x) {
    Tf_PostErrorHelper(pxr::TF_CALL_CONTEXT,
                       pxr::TF_DIAGNOSTIC_RUNTIME_ERROR_TYPE,
                       "MaterialX cycle found: %s\n",
                       x.what());
    return;
  }
  catch (MaterialX::Exception &x) {
    Tf_PostErrorHelper(pxr::TF_CALL_CONTEXT,
                       pxr::TF_DIAGNOSTIC_RUNTIME_ERROR_TYPE,
                       "MaterialX error: %s\n",
                       x.what());
    return;
  }

  if (pxr::UsdPrim materials = stage->GetPrimAtPath(pxr::SdfPath("/MaterialX/Materials"))) {
    if (pxr::UsdPrimSiblingRange children = materials.GetChildren()) {
      if (auto material = pxr::UsdShadeMaterial(*children.begin())) {
        if (pxr::UsdShadeShader mtlx_surface = material.ComputeSurfaceSource(render_contexts)) {
          UsdImagingBuildHdMaterialNetworkFromTerminal(mtlx_surface.GetPrim(),
                                                       pxr::HdMaterialTerminalTokens->surface,
                                                       shader_source_types,
                                                       render_contexts,
                                                       out,
                                                       pxr::UsdTimeCode::Default());
        }
      }
    }
  }
#else
  UNUSED_VARS(mtlx_path, shader_source_types, render_contexts, out);
#endif
}

}  // namespace blender::render::hydra
