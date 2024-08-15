/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup imbuf
 */

#pragma once

struct ImBuf;


/**
 * Create thumbnail for file and returns new ImBuf for thumbnail.
 * \param filepath: File path (but not a library path!) to the thumbnail to be created.
 */
ImBuf *IMB_thumb_win32(const char *file_path, const size_t size);
