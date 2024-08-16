/* SPDX-FileCopyrightText: 2001-2002 NaN Holding BV. All rights reserved.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup imbuf
 */

#include <cmath>

#include "BLI_task.hh"
#include "BLI_utildefines.h"
#include "MEM_guardedalloc.h"

#include "IMB_filter.hh"
#include "IMB_imbuf.hh"
#include "IMB_imbuf_types.hh"
#include "IMB_interp.hh"

#include "BLI_sys_types.h" /* for intptr_t support */

static void imb_half_x_no_alloc(ImBuf *ibuf2, ImBuf *ibuf1)
{
  uchar *p1, *_p1, *dest;
  short a, r, g, b;
  int x, y;
  float af, rf, gf, bf, *p1f, *_p1f, *destf;
  bool do_rect, do_float;

  do_rect = (ibuf1->byte_buffer.data != nullptr);
  do_float = (ibuf1->float_buffer.data != nullptr && ibuf2->float_buffer.data != nullptr);

  _p1 = ibuf1->byte_buffer.data;
  dest = ibuf2->byte_buffer.data;

  _p1f = ibuf1->float_buffer.data;
  destf = ibuf2->float_buffer.data;

  for (y = ibuf2->y; y > 0; y--) {
    p1 = _p1;
    p1f = _p1f;
    for (x = ibuf2->x; x > 0; x--) {
      if (do_rect) {
        a = *(p1++);
        b = *(p1++);
        g = *(p1++);
        r = *(p1++);
        a += *(p1++);
        b += *(p1++);
        g += *(p1++);
        r += *(p1++);
        *(dest++) = a >> 1;
        *(dest++) = b >> 1;
        *(dest++) = g >> 1;
        *(dest++) = r >> 1;
      }
      if (do_float) {
        af = *(p1f++);
        bf = *(p1f++);
        gf = *(p1f++);
        rf = *(p1f++);
        af += *(p1f++);
        bf += *(p1f++);
        gf += *(p1f++);
        rf += *(p1f++);
        *(destf++) = 0.5f * af;
        *(destf++) = 0.5f * bf;
        *(destf++) = 0.5f * gf;
        *(destf++) = 0.5f * rf;
      }
    }
    if (do_rect) {
      _p1 += (ibuf1->x << 2);
    }
    if (do_float) {
      _p1f += (ibuf1->x << 2);
    }
  }
}

ImBuf *IMB_half_x(ImBuf *ibuf1)
{
  ImBuf *ibuf2;

  if (ibuf1 == nullptr) {
    return nullptr;
  }
  if (ibuf1->byte_buffer.data == nullptr && ibuf1->float_buffer.data == nullptr) {
    return nullptr;
  }

  if (ibuf1->x <= 1) {
    return IMB_dupImBuf(ibuf1);
  }

  ibuf2 = IMB_allocImBuf((ibuf1->x) / 2, ibuf1->y, ibuf1->planes, ibuf1->flags);
  if (ibuf2 == nullptr) {
    return nullptr;
  }

  imb_half_x_no_alloc(ibuf2, ibuf1);

  return ibuf2;
}

static void imb_half_y_no_alloc(ImBuf *ibuf2, ImBuf *ibuf1)
{
  uchar *p1, *p2, *_p1, *dest;
  short a, r, g, b;
  int x, y;
  float af, rf, gf, bf, *p1f, *p2f, *_p1f, *destf;

  p1 = p2 = nullptr;
  p1f = p2f = nullptr;

  const bool do_rect = (ibuf1->byte_buffer.data != nullptr);
  const bool do_float = (ibuf1->float_buffer.data != nullptr &&
                         ibuf2->float_buffer.data != nullptr);

  _p1 = ibuf1->byte_buffer.data;
  dest = ibuf2->byte_buffer.data;
  _p1f = (float *)ibuf1->float_buffer.data;
  destf = (float *)ibuf2->float_buffer.data;

  for (y = ibuf2->y; y > 0; y--) {
    if (do_rect) {
      p1 = _p1;
      p2 = _p1 + (ibuf1->x << 2);
    }
    if (do_float) {
      p1f = _p1f;
      p2f = _p1f + (ibuf1->x << 2);
    }
    for (x = ibuf2->x; x > 0; x--) {
      if (do_rect) {
        a = *(p1++);
        b = *(p1++);
        g = *(p1++);
        r = *(p1++);
        a += *(p2++);
        b += *(p2++);
        g += *(p2++);
        r += *(p2++);
        *(dest++) = a >> 1;
        *(dest++) = b >> 1;
        *(dest++) = g >> 1;
        *(dest++) = r >> 1;
      }
      if (do_float) {
        af = *(p1f++);
        bf = *(p1f++);
        gf = *(p1f++);
        rf = *(p1f++);
        af += *(p2f++);
        bf += *(p2f++);
        gf += *(p2f++);
        rf += *(p2f++);
        *(destf++) = 0.5f * af;
        *(destf++) = 0.5f * bf;
        *(destf++) = 0.5f * gf;
        *(destf++) = 0.5f * rf;
      }
    }
    if (do_rect) {
      _p1 += (ibuf1->x << 3);
    }
    if (do_float) {
      _p1f += (ibuf1->x << 3);
    }
  }
}

ImBuf *IMB_half_y(ImBuf *ibuf1)
{
  ImBuf *ibuf2;

  if (ibuf1 == nullptr) {
    return nullptr;
  }
  if (ibuf1->byte_buffer.data == nullptr && ibuf1->float_buffer.data == nullptr) {
    return nullptr;
  }

  if (ibuf1->y <= 1) {
    return IMB_dupImBuf(ibuf1);
  }

  ibuf2 = IMB_allocImBuf(ibuf1->x, (ibuf1->y) / 2, ibuf1->planes, ibuf1->flags);
  if (ibuf2 == nullptr) {
    return nullptr;
  }

  imb_half_y_no_alloc(ibuf2, ibuf1);

  return ibuf2;
}

/* pretty much specific functions which converts uchar <-> ushort but assumes
 * ushort range of 255*255 which is more convenient here
 */
MINLINE void straight_uchar_to_premul_ushort(ushort result[4], const uchar color[4])
{
  ushort alpha = color[3];

  result[0] = color[0] * alpha;
  result[1] = color[1] * alpha;
  result[2] = color[2] * alpha;
  result[3] = alpha * 256;
}

MINLINE void premul_ushort_to_straight_uchar(uchar *result, const ushort color[4])
{
  if (color[3] <= 255) {
    result[0] = unit_ushort_to_uchar(color[0]);
    result[1] = unit_ushort_to_uchar(color[1]);
    result[2] = unit_ushort_to_uchar(color[2]);
    result[3] = unit_ushort_to_uchar(color[3]);
  }
  else {
    ushort alpha = color[3] / 256;

    result[0] = unit_ushort_to_uchar(ushort(color[0] / alpha * 256));
    result[1] = unit_ushort_to_uchar(ushort(color[1] / alpha * 256));
    result[2] = unit_ushort_to_uchar(ushort(color[2] / alpha * 256));
    result[3] = unit_ushort_to_uchar(color[3]);
  }
}

void imb_onehalf_no_alloc(ImBuf *ibuf2, ImBuf *ibuf1)
{
  int x, y;
  const bool do_rect = (ibuf1->byte_buffer.data != nullptr);
  const bool do_float = (ibuf1->float_buffer.data != nullptr) &&
                        (ibuf2->float_buffer.data != nullptr);

  if (do_rect && (ibuf2->byte_buffer.data == nullptr)) {
    imb_addrectImBuf(ibuf2);
  }

  if (ibuf1->x <= 1) {
    imb_half_y_no_alloc(ibuf2, ibuf1);
    return;
  }
  if (ibuf1->y <= 1) {
    imb_half_x_no_alloc(ibuf2, ibuf1);
    return;
  }

  if (do_rect) {
    uchar *cp1, *cp2, *dest;

    cp1 = ibuf1->byte_buffer.data;
    dest = ibuf2->byte_buffer.data;

    for (y = ibuf2->y; y > 0; y--) {
      cp2 = cp1 + (ibuf1->x << 2);
      for (x = ibuf2->x; x > 0; x--) {
        ushort p1i[8], p2i[8], desti[4];

        straight_uchar_to_premul_ushort(p1i, cp1);
        straight_uchar_to_premul_ushort(p2i, cp2);
        straight_uchar_to_premul_ushort(p1i + 4, cp1 + 4);
        straight_uchar_to_premul_ushort(p2i + 4, cp2 + 4);

        desti[0] = (uint(p1i[0]) + p2i[0] + p1i[4] + p2i[4]) >> 2;
        desti[1] = (uint(p1i[1]) + p2i[1] + p1i[5] + p2i[5]) >> 2;
        desti[2] = (uint(p1i[2]) + p2i[2] + p1i[6] + p2i[6]) >> 2;
        desti[3] = (uint(p1i[3]) + p2i[3] + p1i[7] + p2i[7]) >> 2;

        premul_ushort_to_straight_uchar(dest, desti);

        cp1 += 8;
        cp2 += 8;
        dest += 4;
      }
      cp1 = cp2;
      if (ibuf1->x & 1) {
        cp1 += 4;
      }
    }
  }

  if (do_float) {
    float *p1f, *p2f, *destf;

    p1f = ibuf1->float_buffer.data;
    destf = ibuf2->float_buffer.data;
    for (y = ibuf2->y; y > 0; y--) {
      p2f = p1f + (ibuf1->x << 2);
      for (x = ibuf2->x; x > 0; x--) {
        destf[0] = 0.25f * (p1f[0] + p2f[0] + p1f[4] + p2f[4]);
        destf[1] = 0.25f * (p1f[1] + p2f[1] + p1f[5] + p2f[5]);
        destf[2] = 0.25f * (p1f[2] + p2f[2] + p1f[6] + p2f[6]);
        destf[3] = 0.25f * (p1f[3] + p2f[3] + p1f[7] + p2f[7]);
        p1f += 8;
        p2f += 8;
        destf += 4;
      }
      p1f = p2f;
      if (ibuf1->x & 1) {
        p1f += 4;
      }
    }
  }
}

ImBuf *IMB_onehalf(ImBuf *ibuf1)
{
  ImBuf *ibuf2;

  if (ibuf1 == nullptr) {
    return nullptr;
  }
  if (ibuf1->byte_buffer.data == nullptr && ibuf1->float_buffer.data == nullptr) {
    return nullptr;
  }

  if (ibuf1->x <= 1) {
    return IMB_half_y(ibuf1);
  }
  if (ibuf1->y <= 1) {
    return IMB_half_x(ibuf1);
  }

  ibuf2 = IMB_allocImBuf((ibuf1->x) / 2, (ibuf1->y) / 2, ibuf1->planes, ibuf1->flags);
  if (ibuf2 == nullptr) {
    return nullptr;
  }

  imb_onehalf_no_alloc(ibuf2, ibuf1);

  return ibuf2;
}

static ImBuf *scaledownx(ImBuf *ibuf, int newx)
{
  const bool do_rect = (ibuf->byte_buffer.data != nullptr);
  const bool do_float = (ibuf->float_buffer.data != nullptr);
  const size_t rect_size = IMB_get_rect_len(ibuf) * 4;

  uchar *rect, *_newrect, *newrect;
  float *rectf, *_newrectf, *newrectf;
  float sample, add, val[4], nval[4], valf[4], nvalf[4];
  int x, y;

  rectf = _newrectf = newrectf = nullptr;
  rect = _newrect = newrect = nullptr;
  nval[0] = nval[1] = nval[2] = nval[3] = 0.0f;
  nvalf[0] = nvalf[1] = nvalf[2] = nvalf[3] = 0.0f;

  if (!do_rect && !do_float) {
    return ibuf;
  }

  if (do_rect) {
    _newrect = static_cast<uchar *>(MEM_mallocN(sizeof(uchar[4]) * newx * ibuf->y, "scaledownx"));
    if (_newrect == nullptr) {
      return ibuf;
    }
  }
  if (do_float) {
    _newrectf = static_cast<float *>(
        MEM_mallocN(sizeof(float[4]) * newx * ibuf->y, "scaledownxf"));
    if (_newrectf == nullptr) {
      if (_newrect) {
        MEM_freeN(_newrect);
      }
      return ibuf;
    }
  }

  add = (ibuf->x - 0.01) / newx;

  if (do_rect) {
    rect = ibuf->byte_buffer.data;
    newrect = _newrect;
  }
  if (do_float) {
    rectf = ibuf->float_buffer.data;
    newrectf = _newrectf;
  }

  for (y = ibuf->y; y > 0; y--) {
    sample = 0.0f;
    val[0] = val[1] = val[2] = val[3] = 0.0f;
    valf[0] = valf[1] = valf[2] = valf[3] = 0.0f;

    for (x = newx; x > 0; x--) {
      if (do_rect) {
        nval[0] = -val[0] * sample;
        nval[1] = -val[1] * sample;
        nval[2] = -val[2] * sample;
        nval[3] = -val[3] * sample;
      }
      if (do_float) {
        nvalf[0] = -valf[0] * sample;
        nvalf[1] = -valf[1] * sample;
        nvalf[2] = -valf[2] * sample;
        nvalf[3] = -valf[3] * sample;
      }

      sample += add;

      while (sample >= 1.0f) {
        sample -= 1.0f;

        if (do_rect) {
          nval[0] += rect[0];
          nval[1] += rect[1];
          nval[2] += rect[2];
          nval[3] += rect[3];
          rect += 4;
        }
        if (do_float) {
          nvalf[0] += rectf[0];
          nvalf[1] += rectf[1];
          nvalf[2] += rectf[2];
          nvalf[3] += rectf[3];
          rectf += 4;
        }
      }

      if (do_rect) {
        val[0] = rect[0];
        val[1] = rect[1];
        val[2] = rect[2];
        val[3] = rect[3];
        rect += 4;

        newrect[0] = roundf((nval[0] + sample * val[0]) / add);
        newrect[1] = roundf((nval[1] + sample * val[1]) / add);
        newrect[2] = roundf((nval[2] + sample * val[2]) / add);
        newrect[3] = roundf((nval[3] + sample * val[3]) / add);

        newrect += 4;
      }
      if (do_float) {

        valf[0] = rectf[0];
        valf[1] = rectf[1];
        valf[2] = rectf[2];
        valf[3] = rectf[3];
        rectf += 4;

        newrectf[0] = ((nvalf[0] + sample * valf[0]) / add);
        newrectf[1] = ((nvalf[1] + sample * valf[1]) / add);
        newrectf[2] = ((nvalf[2] + sample * valf[2]) / add);
        newrectf[3] = ((nvalf[3] + sample * valf[3]) / add);

        newrectf += 4;
      }

      sample -= 1.0f;
    }
  }

  if (do_rect) {
    // printf("%ld %ld\n", (uchar *)rect - ibuf->byte_buffer.data, rect_size);
    BLI_assert((uchar *)rect - ibuf->byte_buffer.data == rect_size); /* see bug #26502. */

    imb_freerectImBuf(ibuf);
    IMB_assign_byte_buffer(ibuf, _newrect, IB_TAKE_OWNERSHIP);
  }
  if (do_float) {
    // printf("%ld %ld\n", rectf - ibuf->float_buffer.data, rect_size);
    BLI_assert((rectf - ibuf->float_buffer.data) == rect_size); /* see bug #26502. */

    imb_freerectfloatImBuf(ibuf);
    IMB_assign_float_buffer(ibuf, _newrectf, IB_TAKE_OWNERSHIP);
  }

  (void)rect_size; /* UNUSED in release builds */

  ibuf->x = newx;
  return ibuf;
}

static ImBuf *scaledowny(ImBuf *ibuf, int newy)
{
  const bool do_rect = (ibuf->byte_buffer.data != nullptr);
  const bool do_float = (ibuf->float_buffer.data != nullptr);
  const size_t rect_size = IMB_get_rect_len(ibuf) * 4;

  uchar *rect, *_newrect, *newrect;
  float *rectf, *_newrectf, *newrectf;
  float sample, add, val[4], nval[4], valf[4], nvalf[4];
  int x, y, skipx;

  rectf = _newrectf = newrectf = nullptr;
  rect = _newrect = newrect = nullptr;
  nval[0] = nval[1] = nval[2] = nval[3] = 0.0f;
  nvalf[0] = nvalf[1] = nvalf[2] = nvalf[3] = 0.0f;

  if (!do_rect && !do_float) {
    return ibuf;
  }

  if (do_rect) {
    _newrect = static_cast<uchar *>(MEM_mallocN(sizeof(uchar[4]) * newy * ibuf->x, "scaledowny"));
    if (_newrect == nullptr) {
      return ibuf;
    }
  }
  if (do_float) {
    _newrectf = static_cast<float *>(
        MEM_mallocN(sizeof(float[4]) * newy * ibuf->x, "scaledownyf"));
    if (_newrectf == nullptr) {
      if (_newrect) {
        MEM_freeN(_newrect);
      }
      return ibuf;
    }
  }

  add = (ibuf->y - 0.01) / newy;
  skipx = 4 * ibuf->x;

  for (x = skipx - 4; x >= 0; x -= 4) {
    if (do_rect) {
      rect = ibuf->byte_buffer.data + x;
      newrect = _newrect + x;
    }
    if (do_float) {
      rectf = ibuf->float_buffer.data + x;
      newrectf = _newrectf + x;
    }

    sample = 0.0f;
    val[0] = val[1] = val[2] = val[3] = 0.0f;
    valf[0] = valf[1] = valf[2] = valf[3] = 0.0f;

    for (y = newy; y > 0; y--) {
      if (do_rect) {
        nval[0] = -val[0] * sample;
        nval[1] = -val[1] * sample;
        nval[2] = -val[2] * sample;
        nval[3] = -val[3] * sample;
      }
      if (do_float) {
        nvalf[0] = -valf[0] * sample;
        nvalf[1] = -valf[1] * sample;
        nvalf[2] = -valf[2] * sample;
        nvalf[3] = -valf[3] * sample;
      }

      sample += add;

      while (sample >= 1.0f) {
        sample -= 1.0f;

        if (do_rect) {
          nval[0] += rect[0];
          nval[1] += rect[1];
          nval[2] += rect[2];
          nval[3] += rect[3];
          rect += skipx;
        }
        if (do_float) {
          nvalf[0] += rectf[0];
          nvalf[1] += rectf[1];
          nvalf[2] += rectf[2];
          nvalf[3] += rectf[3];
          rectf += skipx;
        }
      }

      if (do_rect) {
        val[0] = rect[0];
        val[1] = rect[1];
        val[2] = rect[2];
        val[3] = rect[3];
        rect += skipx;

        newrect[0] = roundf((nval[0] + sample * val[0]) / add);
        newrect[1] = roundf((nval[1] + sample * val[1]) / add);
        newrect[2] = roundf((nval[2] + sample * val[2]) / add);
        newrect[3] = roundf((nval[3] + sample * val[3]) / add);

        newrect += skipx;
      }
      if (do_float) {

        valf[0] = rectf[0];
        valf[1] = rectf[1];
        valf[2] = rectf[2];
        valf[3] = rectf[3];
        rectf += skipx;

        newrectf[0] = ((nvalf[0] + sample * valf[0]) / add);
        newrectf[1] = ((nvalf[1] + sample * valf[1]) / add);
        newrectf[2] = ((nvalf[2] + sample * valf[2]) / add);
        newrectf[3] = ((nvalf[3] + sample * valf[3]) / add);

        newrectf += skipx;
      }

      sample -= 1.0f;
    }
  }

  if (do_rect) {
    // printf("%ld %ld\n", (uchar *)rect - byte_buffer.data, rect_size);
    BLI_assert((uchar *)rect - ibuf->byte_buffer.data == rect_size); /* see bug #26502. */

    imb_freerectImBuf(ibuf);
    IMB_assign_byte_buffer(ibuf, _newrect, IB_TAKE_OWNERSHIP);
  }
  if (do_float) {
    // printf("%ld %ld\n", rectf - ibuf->float_buffer.data, rect_size);
    BLI_assert((rectf - ibuf->float_buffer.data) == rect_size); /* see bug #26502. */

    imb_freerectfloatImBuf(ibuf);
    IMB_assign_float_buffer(ibuf, _newrectf, IB_TAKE_OWNERSHIP);
  }

  (void)rect_size; /* UNUSED in release builds */

  ibuf->y = newy;
  return ibuf;
}

static ImBuf *scaleupx(ImBuf *ibuf, int newx)
{
  uchar *rect, *_newrect = nullptr, *newrect;
  float *rectf, *_newrectf = nullptr, *newrectf;
  int x, y;
  bool do_rect = false, do_float = false;

  if (ibuf == nullptr) {
    return nullptr;
  }
  if (ibuf->byte_buffer.data == nullptr && ibuf->float_buffer.data == nullptr) {
    return ibuf;
  }

  if (ibuf->byte_buffer.data) {
    do_rect = true;
    _newrect = static_cast<uchar *>(MEM_mallocN(newx * ibuf->y * sizeof(int), "scaleupx"));
    if (_newrect == nullptr) {
      return ibuf;
    }
  }
  if (ibuf->float_buffer.data) {
    do_float = true;
    _newrectf = static_cast<float *>(MEM_mallocN(sizeof(float[4]) * newx * ibuf->y, "scaleupxf"));
    if (_newrectf == nullptr) {
      if (_newrect) {
        MEM_freeN(_newrect);
      }
      return ibuf;
    }
  }

  rect = ibuf->byte_buffer.data;
  rectf = ibuf->float_buffer.data;
  newrect = _newrect;
  newrectf = _newrectf;

  /* Special case, copy all columns, needed since the scaling logic assumes there is at least
   * two rows to interpolate between causing out of bounds read for 1px images, see #70356. */
  if (UNLIKELY(ibuf->x == 1)) {
    if (do_rect) {
      for (y = ibuf->y; y > 0; y--) {
        for (x = newx; x > 0; x--) {
          memcpy(newrect, rect, sizeof(char[4]));
          newrect += 4;
        }
        rect += 4;
      }
    }
    if (do_float) {
      for (y = ibuf->y; y > 0; y--) {
        for (x = newx; x > 0; x--) {
          memcpy(newrectf, rectf, sizeof(float[4]));
          newrectf += 4;
        }
        rectf += 4;
      }
    }
  }
  else {
    const float add = (ibuf->x - 1.001) / (newx - 1.0);
    float sample;

    float val_a, nval_a, diff_a;
    float val_b, nval_b, diff_b;
    float val_g, nval_g, diff_g;
    float val_r, nval_r, diff_r;
    float val_af, nval_af, diff_af;
    float val_bf, nval_bf, diff_bf;
    float val_gf, nval_gf, diff_gf;
    float val_rf, nval_rf, diff_rf;

    val_a = nval_a = diff_a = val_b = nval_b = diff_b = 0;
    val_g = nval_g = diff_g = val_r = nval_r = diff_r = 0;
    val_af = nval_af = diff_af = val_bf = nval_bf = diff_bf = 0;
    val_gf = nval_gf = diff_gf = val_rf = nval_rf = diff_rf = 0;

    for (y = ibuf->y; y > 0; y--) {

      sample = 0;

      if (do_rect) {
        val_a = rect[0];
        nval_a = rect[4];
        diff_a = nval_a - val_a;
        val_a += 0.5f;

        val_b = rect[1];
        nval_b = rect[5];
        diff_b = nval_b - val_b;
        val_b += 0.5f;

        val_g = rect[2];
        nval_g = rect[6];
        diff_g = nval_g - val_g;
        val_g += 0.5f;

        val_r = rect[3];
        nval_r = rect[7];
        diff_r = nval_r - val_r;
        val_r += 0.5f;

        rect += 8;
      }
      if (do_float) {
        val_af = rectf[0];
        nval_af = rectf[4];
        diff_af = nval_af - val_af;

        val_bf = rectf[1];
        nval_bf = rectf[5];
        diff_bf = nval_bf - val_bf;

        val_gf = rectf[2];
        nval_gf = rectf[6];
        diff_gf = nval_gf - val_gf;

        val_rf = rectf[3];
        nval_rf = rectf[7];
        diff_rf = nval_rf - val_rf;

        rectf += 8;
      }
      for (x = newx; x > 0; x--) {
        if (sample >= 1.0f) {
          sample -= 1.0f;

          if (do_rect) {
            val_a = nval_a;
            nval_a = rect[0];
            diff_a = nval_a - val_a;
            val_a += 0.5f;

            val_b = nval_b;
            nval_b = rect[1];
            diff_b = nval_b - val_b;
            val_b += 0.5f;

            val_g = nval_g;
            nval_g = rect[2];
            diff_g = nval_g - val_g;
            val_g += 0.5f;

            val_r = nval_r;
            nval_r = rect[3];
            diff_r = nval_r - val_r;
            val_r += 0.5f;
            rect += 4;
          }
          if (do_float) {
            val_af = nval_af;
            nval_af = rectf[0];
            diff_af = nval_af - val_af;

            val_bf = nval_bf;
            nval_bf = rectf[1];
            diff_bf = nval_bf - val_bf;

            val_gf = nval_gf;
            nval_gf = rectf[2];
            diff_gf = nval_gf - val_gf;

            val_rf = nval_rf;
            nval_rf = rectf[3];
            diff_rf = nval_rf - val_rf;
            rectf += 4;
          }
        }
        if (do_rect) {
          newrect[0] = val_a + sample * diff_a;
          newrect[1] = val_b + sample * diff_b;
          newrect[2] = val_g + sample * diff_g;
          newrect[3] = val_r + sample * diff_r;
          newrect += 4;
        }
        if (do_float) {
          newrectf[0] = val_af + sample * diff_af;
          newrectf[1] = val_bf + sample * diff_bf;
          newrectf[2] = val_gf + sample * diff_gf;
          newrectf[3] = val_rf + sample * diff_rf;
          newrectf += 4;
        }
        sample += add;
      }
    }
  }

  if (do_rect) {
    imb_freerectImBuf(ibuf);
    IMB_assign_byte_buffer(ibuf, _newrect, IB_TAKE_OWNERSHIP);
  }
  if (do_float) {
    imb_freerectfloatImBuf(ibuf);
    IMB_assign_float_buffer(ibuf, _newrectf, IB_TAKE_OWNERSHIP);
  }

  ibuf->x = newx;
  return ibuf;
}

static ImBuf *scaleupy(ImBuf *ibuf, int newy)
{
  uchar *rect, *_newrect = nullptr, *newrect;
  float *rectf, *_newrectf = nullptr, *newrectf;
  int x, y, skipx;
  bool do_rect = false, do_float = false;

  if (ibuf == nullptr) {
    return nullptr;
  }
  if (ibuf->byte_buffer.data == nullptr && ibuf->float_buffer.data == nullptr) {
    return ibuf;
  }

  if (ibuf->byte_buffer.data) {
    do_rect = true;
    _newrect = static_cast<uchar *>(MEM_mallocN(ibuf->x * newy * sizeof(int), "scaleupy"));
    if (_newrect == nullptr) {
      return ibuf;
    }
  }
  if (ibuf->float_buffer.data) {
    do_float = true;
    _newrectf = static_cast<float *>(MEM_mallocN(sizeof(float[4]) * ibuf->x * newy, "scaleupyf"));
    if (_newrectf == nullptr) {
      if (_newrect) {
        MEM_freeN(_newrect);
      }
      return ibuf;
    }
  }

  rect = ibuf->byte_buffer.data;
  rectf = ibuf->float_buffer.data;
  newrect = _newrect;
  newrectf = _newrectf;

  skipx = 4 * ibuf->x;

  /* Special case, copy all rows, needed since the scaling logic assumes there is at least
   * two rows to interpolate between causing out of bounds read for 1px images, see #70356. */
  if (UNLIKELY(ibuf->y == 1)) {
    if (do_rect) {
      for (y = newy; y > 0; y--) {
        memcpy(newrect, rect, sizeof(char) * skipx);
        newrect += skipx;
      }
    }
    if (do_float) {
      for (y = newy; y > 0; y--) {
        memcpy(newrectf, rectf, sizeof(float) * skipx);
        newrectf += skipx;
      }
    }
  }
  else {
    const float add = (ibuf->y - 1.001) / (newy - 1.0);
    float sample;

    float val_a, nval_a, diff_a;
    float val_b, nval_b, diff_b;
    float val_g, nval_g, diff_g;
    float val_r, nval_r, diff_r;
    float val_af, nval_af, diff_af;
    float val_bf, nval_bf, diff_bf;
    float val_gf, nval_gf, diff_gf;
    float val_rf, nval_rf, diff_rf;

    val_a = nval_a = diff_a = val_b = nval_b = diff_b = 0;
    val_g = nval_g = diff_g = val_r = nval_r = diff_r = 0;
    val_af = nval_af = diff_af = val_bf = nval_bf = diff_bf = 0;
    val_gf = nval_gf = diff_gf = val_rf = nval_rf = diff_rf = 0;

    for (x = ibuf->x; x > 0; x--) {
      sample = 0;
      if (do_rect) {
        rect = ibuf->byte_buffer.data + 4 * (x - 1);
        newrect = _newrect + 4 * (x - 1);

        val_a = rect[0];
        nval_a = rect[skipx];
        diff_a = nval_a - val_a;
        val_a += 0.5f;

        val_b = rect[1];
        nval_b = rect[skipx + 1];
        diff_b = nval_b - val_b;
        val_b += 0.5f;

        val_g = rect[2];
        nval_g = rect[skipx + 2];
        diff_g = nval_g - val_g;
        val_g += 0.5f;

        val_r = rect[3];
        nval_r = rect[skipx + 3];
        diff_r = nval_r - val_r;
        val_r += 0.5f;

        rect += 2 * skipx;
      }
      if (do_float) {
        rectf = ibuf->float_buffer.data + 4 * (x - 1);
        newrectf = _newrectf + 4 * (x - 1);

        val_af = rectf[0];
        nval_af = rectf[skipx];
        diff_af = nval_af - val_af;

        val_bf = rectf[1];
        nval_bf = rectf[skipx + 1];
        diff_bf = nval_bf - val_bf;

        val_gf = rectf[2];
        nval_gf = rectf[skipx + 2];
        diff_gf = nval_gf - val_gf;

        val_rf = rectf[3];
        nval_rf = rectf[skipx + 3];
        diff_rf = nval_rf - val_rf;

        rectf += 2 * skipx;
      }

      for (y = newy; y > 0; y--) {
        if (sample >= 1.0f) {
          sample -= 1.0f;

          if (do_rect) {
            val_a = nval_a;
            nval_a = rect[0];
            diff_a = nval_a - val_a;
            val_a += 0.5f;

            val_b = nval_b;
            nval_b = rect[1];
            diff_b = nval_b - val_b;
            val_b += 0.5f;

            val_g = nval_g;
            nval_g = rect[2];
            diff_g = nval_g - val_g;
            val_g += 0.5f;

            val_r = nval_r;
            nval_r = rect[3];
            diff_r = nval_r - val_r;
            val_r += 0.5f;
            rect += skipx;
          }
          if (do_float) {
            val_af = nval_af;
            nval_af = rectf[0];
            diff_af = nval_af - val_af;

            val_bf = nval_bf;
            nval_bf = rectf[1];
            diff_bf = nval_bf - val_bf;

            val_gf = nval_gf;
            nval_gf = rectf[2];
            diff_gf = nval_gf - val_gf;

            val_rf = nval_rf;
            nval_rf = rectf[3];
            diff_rf = nval_rf - val_rf;
            rectf += skipx;
          }
        }
        if (do_rect) {
          newrect[0] = val_a + sample * diff_a;
          newrect[1] = val_b + sample * diff_b;
          newrect[2] = val_g + sample * diff_g;
          newrect[3] = val_r + sample * diff_r;
          newrect += skipx;
        }
        if (do_float) {
          newrectf[0] = val_af + sample * diff_af;
          newrectf[1] = val_bf + sample * diff_bf;
          newrectf[2] = val_gf + sample * diff_gf;
          newrectf[3] = val_rf + sample * diff_rf;
          newrectf += skipx;
        }
        sample += add;
      }
    }
  }

  if (do_rect) {
    imb_freerectImBuf(ibuf);
    IMB_assign_byte_buffer(ibuf, _newrect, IB_TAKE_OWNERSHIP);
  }
  if (do_float) {
    imb_freerectfloatImBuf(ibuf);
    IMB_assign_float_buffer(ibuf, _newrectf, IB_TAKE_OWNERSHIP);
  }

  ibuf->y = newy;
  return ibuf;
}

bool IMB_scaleImBuf(ImBuf *ibuf, uint newx, uint newy)
{
  BLI_assert_msg(newx > 0 && newy > 0, "Images must be at least 1 on both dimensions!");

  if (ibuf == nullptr) {
    return false;
  }
  if (ibuf->byte_buffer.data == nullptr && ibuf->float_buffer.data == nullptr) {
    return false;
  }

  if (newx == ibuf->x && newy == ibuf->y) {
    return false;
  }

  if (newx && (newx < ibuf->x)) {
    scaledownx(ibuf, newx);
  }
  if (newy && (newy < ibuf->y)) {
    scaledowny(ibuf, newy);
  }
  if (newx && (newx > ibuf->x)) {
    scaleupx(ibuf, newx);
  }
  if (newy && (newy > ibuf->y)) {
    scaleupy(ibuf, newy);
  }

  return true;
}

bool IMB_scalefastImBuf(ImBuf *ibuf, uint newx, uint newy)
{
  BLI_assert_msg(newx > 0 && newy > 0, "Images must be at least 1 on both dimensions!");

  uint *rect, *_newrect, *newrect;
  float *rectf, *_newrectf, *newrectf;
  int x, y;
  bool do_float = false, do_rect = false;
  size_t ofsx, ofsy, stepx, stepy;

  rect = nullptr;
  _newrect = nullptr;
  newrect = nullptr;
  rectf = nullptr;
  _newrectf = nullptr;
  newrectf = nullptr;

  if (ibuf == nullptr) {
    return false;
  }
  if (ibuf->byte_buffer.data) {
    do_rect = true;
  }
  if (ibuf->float_buffer.data) {
    do_float = true;
  }
  if (do_rect == false && do_float == false) {
    return false;
  }

  if (newx == ibuf->x && newy == ibuf->y) {
    return false;
  }

  if (do_rect) {
    _newrect = static_cast<uint *>(MEM_mallocN(newx * newy * sizeof(int), "scalefastimbuf"));
    if (_newrect == nullptr) {
      return false;
    }
    newrect = _newrect;
  }

  if (do_float) {
    _newrectf = static_cast<float *>(
        MEM_mallocN(sizeof(float) * ibuf->channels * newx * newy, "scalefastimbuf f"));
    if (_newrectf == nullptr) {
      if (_newrect) {
        MEM_freeN(_newrect);
      }
      return false;
    }
    newrectf = _newrectf;
  }

  stepx = round(65536.0 * (ibuf->x - 1.0) / (newx - 1.0));
  stepy = round(65536.0 * (ibuf->y - 1.0) / (newy - 1.0));
  ofsy = 32768;

  for (y = newy; y > 0; y--, ofsy += stepy) {
    if (do_rect) {
      rect = (uint *)ibuf->byte_buffer.data;
      rect += (ofsy >> 16) * ibuf->x;
      ofsx = 32768;

      for (x = newx; x > 0; x--, ofsx += stepx) {
        *newrect++ = rect[ofsx >> 16];
      }
    }

    if (do_float) {
      rectf = ibuf->float_buffer.data;
      rectf += size_t(ofsy >> 16) * ibuf->x * ibuf->channels;
      ofsx = 32768;

      for (x = newx; x > 0; x--, ofsx += stepx) {
        float *pixel = &rectf[size_t(ofsx >> 16) * ibuf->channels];
        for (int c = 0; c < ibuf->channels; ++c) {
          *newrectf++ = pixel[c];
        }
      }
    }
  }

  if (do_rect) {
    imb_freerectImBuf(ibuf);
    IMB_assign_byte_buffer(ibuf, reinterpret_cast<uint8_t *>(_newrect), IB_TAKE_OWNERSHIP);
  }

  if (do_float) {
    imb_freerectfloatImBuf(ibuf);
    IMB_assign_float_buffer(ibuf, reinterpret_cast<float *>(_newrectf), IB_TAKE_OWNERSHIP);
  }

  ibuf->x = newx;
  ibuf->y = newy;
  return true;
}

void IMB_scaleImBuf_threaded(ImBuf *ibuf, uint newx, uint newy)
{
  using namespace blender;
  using namespace blender::imbuf;
  BLI_assert_msg(newx > 0 && newy > 0, "Images must be at least 1 on both dimensions!");

  /* Create destination buffers. */
  uchar *dst_byte_buffer = nullptr;
  if (ibuf->byte_buffer.data) {
    dst_byte_buffer = static_cast<uchar *>(
        MEM_mallocN(sizeof(uchar) * 4 * newx * newy, "threaded scale byte buffer"));
  }
  float *dst_float_buffer = nullptr;
  if (ibuf->float_buffer.data) {
    dst_float_buffer = static_cast<float *>(
        MEM_mallocN(sizeof(float) * ibuf->channels * newx * newy, "threaded scale float buffer"));
  }

  /* Threaded processing. */
  threading::parallel_for(IndexRange(newy), 32, [&](IndexRange y_range) {
    float factor_x = float(ibuf->x) / newx;
    float factor_y = float(ibuf->y) / newy;

    for (const int y : y_range) {
      float v = (float(y) + 0.5f) * factor_y - 0.5f;
      for (int x = 0; x < newx; x++) {
        float u = (float(x) + 0.5f) * factor_x - 0.5f;
        int64_t offset = int64_t(y) * newx + x;

        if (dst_byte_buffer) {
          interpolate_bilinear_byte(ibuf, dst_byte_buffer + 4 * offset, u, v);
        }

        if (dst_float_buffer) {
          float *pixel = dst_float_buffer + ibuf->channels * offset;
          blender::math::interpolate_bilinear_fl(
              ibuf->float_buffer.data, pixel, ibuf->x, ibuf->y, ibuf->channels, u, v);
        }
      }
    }
  });

  /* Alter the image. */
  ibuf->x = newx;
  ibuf->y = newy;

  if (ibuf->byte_buffer.data) {
    imb_freerectImBuf(ibuf);
    IMB_assign_byte_buffer(ibuf, dst_byte_buffer, IB_TAKE_OWNERSHIP);
  }
  if (ibuf->float_buffer.data) {
    imb_freerectfloatImBuf(ibuf);
    IMB_assign_float_buffer(ibuf, dst_float_buffer, IB_TAKE_OWNERSHIP);
  }
}
