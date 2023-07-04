/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "IMB_colormanagement.h"
#include "IMB_filetype.h"
#include "IMB_imbuf_types.h"

/* Custom flags for NanoSVG. */
#define NANOSVG_ALL_COLOR_KEYWORDS
#define NANOSVG_IMPLEMENTATION
#define NANOSVGRAST_IMPLEMENTATION

#include "../../../extern/nanosvg/nanosvgrast.h"

extern "C" {

ImBuf *imb_load_filepath_thumbnail_svg(const char *filepath,
                                       const int /* flags */,
                                       const size_t max_thumb_size,
                                       char colorspace[],
                                       size_t *r_width,
                                       size_t *r_height)
{
  NSVGimage *image = nsvgParseFromFile(filepath, "px", 96.0f);

  if (image == nullptr) {
    return nullptr;
  }

  if (image->width == 0 || image->height == 0) {
    nsvgDelete(image);
    return nullptr;
  }

	int w = int(image->width);
  int h = int(image->height);

  /* Return full size of the image. */
  *r_width = size_t(w);
  *r_height = size_t(h);

	NSVGrasterizer *rast = nsvgCreateRasterizer();
  if (rast == nullptr) {
    nsvgDelete(image);
    return nullptr;
  }

  colorspace_set_default_role(colorspace, IM_MAX_SPACE, COLOR_ROLE_DEFAULT_BYTE);

  const float scale = float(max_thumb_size) / MAX2(w, h);
  const int dest_w = MAX2(int(w * scale), 1);
  const int dest_h = MAX2(int(h * scale), 1);

  ImBuf *ibuf = IMB_allocImBuf(dest_w, dest_h, 32, IB_rect);
  if (ibuf != nullptr) {
    nsvgRasterize(rast, image, 0, 0, scale, ibuf->byte_buffer.data, dest_w, dest_h, dest_w * 4);
    nsvgDeleteRasterizer(rast);
    nsvgDelete(image);
    IMB_flipy(ibuf);
  }

  return ibuf;
}
}
