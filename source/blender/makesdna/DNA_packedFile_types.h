/* SPDX-FileCopyrightText: 2001-2002 NaN Holding BV. All rights reserved.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup DNA
 */

#pragma once

#include "BLI_implicit_sharing.h"

typedef struct PackedFile {
  int size;
  int seek;
  const void *data;
  const ImplicitSharingInfoHandle *sharing_info;
} PackedFile;
