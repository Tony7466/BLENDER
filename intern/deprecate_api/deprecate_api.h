/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Typically not directly included. */
#pragma once

#ifdef __GNUC__
#  define _DEPRECATE_ATTR __attribute__((deprecated))
#elif defined(_MSC_VER)
#  define _DEPRECATE_ATTR __declspec(deprecated)
#else
#  define _DEPRECATE_ATTR
#endif

#ifdef __cplusplus
#  define _DEPRECATE_CXX_NOEXCEPT noexcept
#else
#  define _DEPRECATE_CXX_NOEXCEPT
#endif

#ifdef __cplusplus
extern "C" {
#endif

_DEPRECATE_ATTR
extern char *strcat(char *, const char *)
#ifndef _MSC_VER
    _DEPRECATE_CXX_NOEXCEPT
#endif
    ;

#ifdef __cplusplus
};
#endif
