/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup wm
 *
 * Interactions with the underlying platform.
 */

#include "BLI_string.h"

#include "WM_api.hh" /* Own include. */

#ifdef WIN32
#  include "BLI_winstuff.h"
#elif defined(__APPLE__)
/* Pass. */
#else
#  include "BKE_context.hh"

#  include "BPY_extern_run.h"
#endif

/* -------------------------------------------------------------------- */
/** \name Register File Assosiation
 * \{ */

bool WM_platform_assosiate_set(bool do_register, bool all_users)
{
  bool result = false;
#ifdef WIN32
  {
    if (all_users) {
      if (do_register) {
        result = BLI_windows_execute_self("--register-allusers", true, true, true);
      }
      else {
        result = BLI_windows_execute_self("--unregister-allusers", true, true, true);
      }
    }
    else {
      if (do_register) {
        result = BLI_windows_register_blend_extension(false);
      }
      else {
        result = BLI_windows_unregister_blend_extension(false);
      }
    }
#elif defined(__APPLE__)
  /* Pass. */
#else
  {
    const char *imports[] = {"_bpy_internal", "_bpy_internal.freedesktop", nullptr};
    char expr_buf[128];

    SNPRINTF(expr_buf,
             "_bpy_internal.freedesktop.%s(all_users=%d)",
             do_register ? "register" : "unregister",
             int(all_users));

    /* NOTE: this could be null, however the running a script without `bpy.context` access
     * is a rare enough situation that it's better to keep this a requirement of the API and
     * pass in a temporary context instead of making an exception for this one case. */
    bContext *C_temp = CTX_create();
    intptr_t value = 0;
    BPY_run_string_as_intptr(C_temp, imports, expr_buf, nullptr, &value);
    result = bool(value);
    CTX_free(C_temp);
  }
#endif
    return result;
  }

  /** \} */
