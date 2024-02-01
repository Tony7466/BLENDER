/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_tempfile.h"

#include "BLI_fileops.h"
#include "BLI_path_util.h"
#include "BLI_string.h"

void BLI_temp_directory_path_get(char *temp_directory, const size_t buffer_size)
{
  temp_directory[0] = '\0';

  const char *env_vars[] = {
#ifdef WIN32
      "TEMP",
#else
      /* XDG Base Directory Specification
       * Suggested default of HOME/.local/state/ */
      "XDG_STATE_HOME",
#endif
  };

  for (int i = 0; i < ARRAY_SIZE(env_vars); i++) {
    const char *tmp = BLI_getenv(env_vars[i]);
    if (tmp && (tmp[0] != '\0') && BLI_is_dir(tmp)) {
      BLI_strncpy(temp_directory, tmp, buffer_size);
      BLI_strncat(temp_directory, "/blender/", FILE_MAX);
      break;
    }
  }

  if (temp_directory[0] == '\0') {
    BLI_strncpy(temp_directory, BLI_getenv("HOME"), buffer_size);
    BLI_strncat(temp_directory, "/.local/state/blender/", FILE_MAX);
  }
  else {
    /* Add a trailing slash if needed. */
    BLI_path_slash_ensure(temp_directory, buffer_size);
  }

  BLI_dir_create_recursive(temp_directory);
}
