/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "testing/testing.h"
#include "tests/blendfile_loading_base_test.h"

#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/stage.h>

#include "BKE_appdir.h"
#include "BKE_context.h"
#include "BKE_main.h"
#include "BLI_fileops.h"
#include "BLI_path_util.h"
#include "BLO_readfile.h"

#include "DEG_depsgraph.h"

#include "WM_api.h"

#include "usd.h"
#include "usd_tests_common.h"

namespace blender::io::usd {

const StringRefNull usdz_export_test_filename = "usd/usdz_export_test.blend";
char temp_dir[FILE_MAX];
char output_filename[FILE_MAX];

class UsdUsdzExportTest : public BlendfileLoadingBaseTest {
 protected:
  struct bContext *context = nullptr;

 public:
  bool load_file_and_depsgraph(const StringRefNull &filepath,
                               const eEvaluationMode eval_mode = DAG_EVAL_VIEWPORT)
  {
    if (!blendfile_load(filepath.c_str())) {
      return false;
    }
    depsgraph_create(eval_mode);

    context = CTX_create();
    CTX_data_main_set(context, bfile->main);
    CTX_data_scene_set(context, bfile->curscene);

    return true;
  }

  virtual void SetUp() override
  {
    BlendfileLoadingBaseTest::SetUp();
    std::string usd_plugin_path = register_usd_plugins_for_tests();
    if (usd_plugin_path.empty()) {
      FAIL();
    }

    char original_wd[FILE_MAX];
    BLI_current_working_dir(original_wd, FILE_MAX);

    BLI_path_join(temp_dir, FILE_MAX, original_wd, "test_temp_dir");
    BLI_dir_create_recursive(temp_dir);
    BKE_tempdir_init(temp_dir);

    const std::string &test_assets_dir = blender::tests::flags_test_asset_dir();
    BLI_path_join(output_filename, FILE_MAX, test_assets_dir.c_str(), "usd/output_новый.usdz");
  }

  virtual void TearDown() override
  {
    BlendfileLoadingBaseTest::TearDown();
    CTX_free(context);
    context = nullptr;

    if (BLI_exists(output_filename)) {
      BLI_delete(output_filename, false, false);
      BLI_delete(temp_dir, true, true);
    }
  }
};

TEST_F(UsdUsdzExportTest, usdz_export)
{
  if (!load_file_and_depsgraph(usdz_export_test_filename)) {
    ADD_FAILURE();
    return;
  }

  /* File sanity check. */
  EXPECT_EQ(BLI_listbase_count(&bfile->main->objects), 4);

  USDExportParams params{};

  bool result = USD_export(context, output_filename, &params, false);
  EXPECT_TRUE(result);

  pxr::UsdStageRefPtr stage = pxr::UsdStage::Open(output_filename);
  EXPECT_TRUE(bool(stage));

  std::string prim_name = pxr::TfMakeValidIdentifier("Cube");
  pxr::UsdPrim test_prim = stage->GetPrimAtPath(pxr::SdfPath("/Cube/" + prim_name));
  EXPECT_TRUE(bool(test_prim));

  prim_name = pxr::TfMakeValidIdentifier("Cylinder");
  test_prim = stage->GetPrimAtPath(pxr::SdfPath("/Cylinder/" + prim_name));
  EXPECT_TRUE(bool(test_prim));

  prim_name = pxr::TfMakeValidIdentifier("Icosphere");
  test_prim = stage->GetPrimAtPath(pxr::SdfPath("/Icosphere/" + prim_name));
  EXPECT_TRUE(bool(test_prim));

  prim_name = pxr::TfMakeValidIdentifier("Sphere");
  test_prim = stage->GetPrimAtPath(pxr::SdfPath("/Sphere/" + prim_name));
  EXPECT_TRUE(bool(test_prim));
}

}  // namespace blender::io::usd