
#include "pxr/usd/usd/prim.h"
#include <pxr/pxr.h>
#include <pxr/usd/sdf/layer.h>
#include <pxr/usd/usd/primRange.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usd/stageCache.h>
#include <pxr/usd/usd/stageCacheContext.h>
#include <pxr/usd/usdGeom/metrics.h>
#include <pxr/usd/usdGeom/sphere.h>
#include <pxr/usd/usdGeom/tokens.h>
#include <pxr/usd/usdGeom/xform.h>
#include <pxr/usd/usdGeom/xformCommonAPI.h>

#include "DNA_usd_stage_types.h"

#include "usd_stage_utils.hh"


/* ====================================================================== */
/* the global stage cache */
pxr::UsdStageCache stage_cache;


/* ====================================================================== */
void usd_stage_clear_cache() {
  stage_cache.Clear();
}

pxr::UsdStageRefPtr get_pxr_stage_for_path(const char* filepath) {
  pxr::UsdStageRefPtr stage = pxr::UsdStage::Open(filepath);
  if (stage) {
    stage_cache.Insert(stage);
    return stage;
  }

  return nullptr;
}

pxr::UsdStageRefPtr usd_stage_get_pxr_stage(const USDStage* stage) {
  return get_pxr_stage_for_path(stage->resolved_filepath);
}

/*
 * !TODO(kiki):
 * This is an obvious mistake.  I should be using UsdStageCacheContext
 * in a struct passed back with the stage pointer itself so that it's
 * more of a smart pointer setup.  I'll fix it later.
 */
bool usd_stage_remove_from_cache(const char* filepath) {
  std::string stage_path(filepath);

  // Iterate over the cached stages and check their paths
  for (const auto& stage : stage_cache.GetAllStages()) {
    if (stage->GetRootLayer()->GetIdentifier() == stage_path) {
      stage_cache.Erase(stage);
      return true;
    }
  }

  return false;  // The stage is not in the cache
}
