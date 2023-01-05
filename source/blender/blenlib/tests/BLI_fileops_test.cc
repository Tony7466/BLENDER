/* SPDX-License-Identifier: Apache-2.0 */

#include "BLI_fileops.hh"
#include "BLI_path_util.h"
#include "BLI_string.h"

#include "testing/testing.h"

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
  char original_wd[FILE_MAX];
  BLI_current_working_dir(original_wd, FILE_MAX);

  char temp_wd[FILE_MAX];
  BLI_path_join(temp_wd, FILE_MAX, original_wd, "test_temp");

  if (BLI_exists(temp_wd)) {
    BLI_delete(temp_wd, true, false);
  }

  bool result = BLI_change_working_dir(temp_wd);
  ASSERT_FALSE(result);

  BLI_dir_create_recursive(temp_wd);

  result = BLI_change_working_dir(temp_wd);
  ASSERT_TRUE(result);

  char cwd[FILE_MAX];
  BLI_current_working_dir(cwd, FILE_MAX);

  ASSERT_TRUE(BLI_path_cmp(cwd, temp_wd) == 0);

  result = BLI_change_working_dir(original_wd);
  ASSERT_TRUE(result);

  BLI_delete(temp_wd, true, false);
}

}  // namespace blender::tests
