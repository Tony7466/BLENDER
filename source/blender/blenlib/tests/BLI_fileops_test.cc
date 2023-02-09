/* SPDX-License-Identifier: Apache-2.0 */

#include "testing/testing.h"

#include "BLI_fileops.hh"
#include "BLI_path_util.h"
#include "BLI_string.h"
#include "BLI_threads.h"

namespace blender::tests {

TEST(fileops, fstream_open_string_filename)
{
  const std::string test_files_dir = blender::tests::flags_test_asset_dir();
  if (test_files_dir.empty()) {
    FAIL();
  }

  const std::string filepath = test_files_dir + "/asset_library/новый/blender_assets.cats.txt";
  fstream in(filepath, std::ios_base::in);
  ASSERT_TRUE(in.is_open()) << "could not open " << filepath;
  in.close(); /* This should not crash. */

  /* Reading the file not tested here. That's deferred to `std::fstream` anyway. */
}

TEST(fileops, fstream_open_charptr_filename)
{
  const std::string test_files_dir = blender::tests::flags_test_asset_dir();
  if (test_files_dir.empty()) {
    FAIL();
  }

  const std::string filepath_str = test_files_dir + "/asset_library/новый/blender_assets.cats.txt";
  const char *filepath = filepath_str.c_str();
  fstream in(filepath, std::ios_base::in);
  ASSERT_TRUE(in.is_open()) << "could not open " << filepath;
  in.close(); /* This should not crash. */

  /* Reading the file not tested here. That's deferred to `std::fstream` anyway. */
}

TEST(fileops, change_working_directory)
{
  BLI_threadapi_init();

  char original_wd[FILE_MAX];
  BLI_current_working_dir(original_wd, FILE_MAX);

  const std::string test_temp_dir = blender::tests::flags_test_asset_dir() + "/" +
                                    "fileops_test_temp_новый";

  if (BLI_exists(test_temp_dir.c_str())) {
    BLI_delete(test_temp_dir.c_str(), true, false);
  }

  bool result = BLI_change_working_dir(test_temp_dir.c_str());
  ASSERT_FALSE(result);

  BLI_dir_create_recursive(test_temp_dir.c_str());

  result = BLI_change_working_dir(test_temp_dir.c_str());
  ASSERT_TRUE(result);

  char cwd[FILE_MAX];
  BLI_current_working_dir(cwd, FILE_MAX);

  ASSERT_TRUE(BLI_path_cmp_normalized(cwd, test_temp_dir.c_str()) == 0);

  result = BLI_change_working_dir(original_wd);
  ASSERT_TRUE(result);

  BLI_delete(test_temp_dir.c_str(), true, false);

  BLI_threadapi_exit();
}

}  // namespace blender::tests
