/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_utildefines.h"

#ifdef __cplusplus
extern "C" {
#endif

/** \file
 * \ingroup imbuf
 */

/* WARNING: Keep explicit value assignments here,
 * this file is included in areas where not all format defines are set
 * (e.g. intern/dds only get WITH_DDS, even if TIFF, HDR etc are also defined).
 * See #46524. */

/** #ImBuf.ftype flag, main image types. */
enum eImbFileType {
  IMB_FTYPE_NONE = 0,
  IMB_FTYPE_PNG = 1,
  IMB_FTYPE_TGA = 2,
  IMB_FTYPE_JPG = 3,
  IMB_FTYPE_BMP = 4,
  IMB_FTYPE_OPENEXR = 5,
  IMB_FTYPE_IMAGIC = 6,
  IMB_FTYPE_PSD = 7,
#ifdef WITH_OPENJPEG
  IMB_FTYPE_JP2 = 8,
#endif
  IMB_FTYPE_RADHDR = 9,
  IMB_FTYPE_TIF = 10,
#ifdef WITH_CINEON
  IMB_FTYPE_CINEON = 11,
  IMB_FTYPE_DPX = 12,
#endif

  IMB_FTYPE_DDS = 13,
#ifdef WITH_WEBP
  IMB_FTYPE_WEBP = 14,
#endif
};

typedef enum IMB_Timecode_Type {
  /** Don't use time-code files at all. Use FFmpeg API to seek to PTS calculated on the fly. */
  IMB_TC_NONE = 0,
  /**
   * Use movie timestamp, which is converted to frame number based on frame rate.
   * Simplified formula is `frameno = time * FPS`. Note, that there may be a frame
   * between say frame 100 and 101. As well as frame may be missing between say
   * frames 100 and 102.
   */
  IMB_TC_INVERSE_MAPPING = 1,
  /**
   * Map each frame in video stream to unique consecutive frame number ordered from first to last.
   */
  IMB_TC_UNIQUE_MAPPING = 2,
  IMB_TC_MAX_SLOT = 2,
} IMB_Timecode_Type;

typedef enum IMB_Proxy_Size {
  IMB_PROXY_NONE = 0,
  IMB_PROXY_25 = 1,
  IMB_PROXY_50 = 2,
  IMB_PROXY_75 = 4,
  IMB_PROXY_100 = 8,
  IMB_PROXY_MAX_SLOT = 4,
} IMB_Proxy_Size;
ENUM_OPERATORS(IMB_Proxy_Size, IMB_PROXY_100);

#ifdef __cplusplus
}
#endif
