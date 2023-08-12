/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bli
 */

#pragma once

#include "BLI_compiler_attrs.h"

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------- */
/** \name Environment Variable Access
 * \{ */

/**
 * Sets the specified environment variable to the specified value,
 * and clears it if `val == NULL`.
 */
void BLI_setenv(const char *env, const char *val) ATTR_NONNULL(1);
/**
 * Only set an environment variable if already not there.
 * Like Unix `setenv(env, val, 0);`
 *
 * (not used anywhere).
 */
void BLI_setenv_if_new(const char *env, const char *val) ATTR_NONNULL(1);
/**
 * Get an environment variable, result has to be used immediately.
 *
 * On windows #getenv gets its variables from a static copy of the environment variables taken at
 * process start-up, causing it to not pick up on environment variables created during runtime.
 * This function uses an alternative method to get environment variables that does pick up on
 * runtime environment variables. The result will be UTF-8 encoded.
 */
const char *BLI_getenv(const char *env) ATTR_NONNULL(1) ATTR_WARN_UNUSED_RESULT;

/** \} */

#ifdef __cplusplus
}
#endif
