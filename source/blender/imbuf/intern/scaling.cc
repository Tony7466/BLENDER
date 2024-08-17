/* SPDX-FileCopyrightText: 2001-2002 NaN Holding BV. All rights reserved.
 * SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup imbuf
 */

#include <cmath>

#include "BLI_math_vector.hh"
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

static void alloc_scale_dst_buffers(
    const ImBuf *ibuf, uint newx, uint newy, blender::uchar4 **r_dst_byte, float **r_dst_float)
{
  *r_dst_byte = nullptr;
  if (ibuf->byte_buffer.data != nullptr) {
    *r_dst_byte = static_cast<blender::uchar4 *>(
        MEM_mallocN(sizeof(blender::uchar4) * newx * newy, "scale_buf_byte"));
    if (*r_dst_byte == nullptr) {
      return;
    }
  }
  *r_dst_float = nullptr;
  if (ibuf->float_buffer.data != nullptr) {
    *r_dst_float = static_cast<float *>(
        MEM_mallocN(sizeof(float) * ibuf->channels * newx * newy, "scale_buf_float"));
    if (*r_dst_float == nullptr) {
      if (*r_dst_byte) {
        MEM_freeN(*r_dst_byte);
      }
      return;
    }
  }
}

static blender::float4 load_pixel(const float *ptr, int channels)
{
  blender::float4 pix(0.0f);
  memcpy(&pix, ptr, channels * sizeof(float));
  return pix;
}

static void store_pixel(float *ptr, blender::float4 pix, int channels)
{
  memcpy(ptr, &pix, channels * sizeof(float));
}

static void scale_down_x(ImBuf *ibuf, int newx)
{
  using namespace blender;
  uchar4 *dst_byte = nullptr;
  float *dst_float = nullptr;
  alloc_scale_dst_buffers(ibuf, newx, ibuf->y, &dst_byte, &dst_float);
  if (dst_byte == nullptr && dst_float == nullptr) {
    return;
  }

  const float add = (ibuf->x - 0.01f) / newx;
  const float inv_add = 1.0f / add;

  /* Byte pixels. */
  if (dst_byte != nullptr) {
    const uchar4 *src_byte = reinterpret_cast<const uchar4 *>(ibuf->byte_buffer.data);
    uchar4 *dst_byte_ptr = dst_byte;
    for (int y = ibuf->y; y > 0; y--) {
      float sample = 0.0f;
      float4 val(0.0f);

      for (int x = newx; x > 0; x--) {
        float4 nval = -val * sample;
        sample += add;
        while (sample >= 1.0f) {
          sample -= 1.0f;
          nval += float4(*src_byte);
          src_byte++;
        }

        val = float4(*src_byte);
        src_byte++;

        float4 pix = math::round((nval + sample * val) * inv_add);
        *dst_byte_ptr = uchar4(pix);
        dst_byte_ptr++;

        sample -= 1.0f;
      }
    }
    BLI_assert((uchar *)src_byte - ibuf->byte_buffer.data ==
               IMB_get_rect_len(ibuf) * 4); /* see bug #26502. */
    imb_freerectImBuf(ibuf);
    IMB_assign_byte_buffer(ibuf, reinterpret_cast<uint8_t *>(dst_byte), IB_TAKE_OWNERSHIP);
  }

  /* Float pixels. */
  if (dst_float != nullptr) {
    const int channels = ibuf->channels;
    const float *src_float = ibuf->float_buffer.data;
    float *dst_float_ptr = dst_float;
    for (int y = ibuf->y; y > 0; y--) {
      float sample = 0.0f;
      float4 val(0.0f);

      for (int x = newx; x > 0; x--) {
        float4 nval = -val * sample;
        sample += add;
        while (sample >= 1.0f) {
          sample -= 1.0f;

          nval += load_pixel(src_float, channels);
          src_float += channels;
        }

        val = load_pixel(src_float, channels);
        src_float += channels;
        float4 pix = (nval + sample * val) * inv_add;
        store_pixel(dst_float_ptr, pix, channels);
        dst_float_ptr += channels;

        sample -= 1.0f;
      }
    }
    BLI_assert((src_float - ibuf->float_buffer.data) ==
               IMB_get_rect_len(ibuf) * channels); /* see bug #26502. */
    imb_freerectfloatImBuf(ibuf);
    IMB_assign_float_buffer(ibuf, dst_float, IB_TAKE_OWNERSHIP);
  }

  ibuf->x = newx;
}

static void scale_down_y(ImBuf *ibuf, int newy)
{
  using namespace blender;
  uchar4 *dst_byte = nullptr;
  float *dst_float = nullptr;
  alloc_scale_dst_buffers(ibuf, ibuf->x, newy, &dst_byte, &dst_float);
  if (dst_byte == nullptr && dst_float == nullptr) {
    return;
  }

  const float add = (ibuf->y - 0.01f) / newy;
  const float inv_add = 1.0f / add;

  /* Byte pixels. */
  if (dst_byte != nullptr) {
    const uchar4 *src_byte = nullptr;
    for (int x = ibuf->x - 1; x >= 0; x--) {
      src_byte = reinterpret_cast<const uchar4 *>(ibuf->byte_buffer.data) + x;
      uchar4 *dst_byte_ptr = dst_byte + x;

      float sample = 0.0f;
      float4 val(0.0f);

      for (int y = newy; y > 0; y--) {
        float4 nval = -val * sample;
        sample += add;
        while (sample >= 1.0f) {
          sample -= 1.0f;
          nval += float4(*src_byte);
          src_byte += ibuf->x;
        }

        val = float4(*src_byte);
        src_byte += ibuf->x;

        float4 pix = math::round((nval + sample * val) * inv_add);
        *dst_byte_ptr = uchar4(pix);
        dst_byte_ptr += ibuf->x;

        sample -= 1.0f;
      }
    }
    BLI_assert((uchar *)src_byte - ibuf->byte_buffer.data ==
               IMB_get_rect_len(ibuf) * 4); /* see bug #26502. */
    imb_freerectImBuf(ibuf);
    IMB_assign_byte_buffer(ibuf, reinterpret_cast<uint8_t *>(dst_byte), IB_TAKE_OWNERSHIP);
  }

  /* Float pixels. */
  if (dst_float != nullptr) {
    const int channels = ibuf->channels;
    const float *src_float = nullptr;
    for (int x = ibuf->x - 1; x >= 0; x--) {
      src_float = ibuf->float_buffer.data + x * channels;
      float *dst_float_ptr = dst_float + x * channels;

      float sample = 0.0f;
      float4 val(0.0f);

      for (int y = newy; y > 0; y--) {
        float4 nval = -val * sample;
        sample += add;
        while (sample >= 1.0f) {
          sample -= 1.0f;
          nval += load_pixel(src_float, channels);
          src_float += ibuf->x * channels;
        }

        val = load_pixel(src_float, channels);
        src_float += ibuf->x * channels;
        float4 pix = (nval + sample * val) * inv_add;
        store_pixel(dst_float_ptr, pix, channels);
        dst_float_ptr += ibuf->x * channels;

        sample -= 1.0f;
      }
    }
    BLI_assert((src_float - ibuf->float_buffer.data) ==
               IMB_get_rect_len(ibuf) * channels); /* see bug #26502. */
    imb_freerectfloatImBuf(ibuf);
    IMB_assign_float_buffer(ibuf, dst_float, IB_TAKE_OWNERSHIP);
  }

  ibuf->y = newy;
}

static void scale_up_x(ImBuf *ibuf, int newx)
{
  using namespace blender;
  uchar4 *dst_byte = nullptr;
  float *dst_float = nullptr;
  alloc_scale_dst_buffers(ibuf, newx, ibuf->y, &dst_byte, &dst_float);
  if (dst_byte == nullptr && dst_float == nullptr) {
    return;
  }

  const float add = (ibuf->x - 1.001f) / (newx - 1.0f);

  /* Byte pixels. */
  if (dst_byte != nullptr) {
    const uchar4 *src_byte = reinterpret_cast<const uchar4 *>(ibuf->byte_buffer.data);

    /* Special case: source is 1px wide (see #70356). */
    if (UNLIKELY(ibuf->x == 1)) {
      for (int y = ibuf->y; y > 0; y--) {
        for (int x = newx; x > 0; x--) {
          *dst_byte = *src_byte;
          dst_byte++;
        }
        src_byte++;
      }
    }
    else {
      uchar4 *dst_byte_ptr = dst_byte;
      for (int y = ibuf->y; y > 0; y--) {
        float sample = 0;
        float4 val = float4(src_byte[0]);
        float4 nval = float4(src_byte[1]);
        float4 diff = nval - val;
        val += 0.5f;
        src_byte += 2;
        for (int x = newx; x > 0; x--) {
          if (sample >= 1.0f) {
            sample -= 1.0f;
            val = nval;
            nval = float4(src_byte[0]);
            diff = nval - val;
            val += 0.5f;
            src_byte++;
          }
          float4 pix = val + sample * diff;
          *dst_byte_ptr = uchar4(pix);
          dst_byte_ptr++;
          sample += add;
        }
      }
    }
    imb_freerectImBuf(ibuf);
    IMB_assign_byte_buffer(ibuf, reinterpret_cast<uint8_t *>(dst_byte), IB_TAKE_OWNERSHIP);
  }

  /* Float pixels. */
  if (dst_float != nullptr) {
    const float *src_float = ibuf->float_buffer.data;

    /* Special case: source is 1px wide (see #70356). */
    if (UNLIKELY(ibuf->x == 1)) {
      for (int y = ibuf->y; y > 0; y--) {
        for (int x = newx; x > 0; x--) {
          memcpy(dst_float, src_float, sizeof(float) * ibuf->channels);
          dst_float += ibuf->channels;
        }
        src_float += ibuf->channels;
      }
    }
    else {
      float *dst_float_ptr = dst_float;
      for (int y = ibuf->y; y > 0; y--) {
        float sample = 0;
        float4 val = load_pixel(src_float, ibuf->channels);
        float4 nval = load_pixel(src_float + ibuf->channels, ibuf->channels);
        float4 diff = nval - val;
        src_float += ibuf->channels * 2;
        for (int x = newx; x > 0; x--) {
          if (sample >= 1.0f) {
            sample -= 1.0f;
            val = nval;
            nval = load_pixel(src_float, ibuf->channels);
            diff = nval - val;
            src_float += ibuf->channels;
          }
          float4 pix = val + sample * diff;
          store_pixel(dst_float_ptr, pix, ibuf->channels);
          dst_float_ptr += ibuf->channels;
          sample += add;
        }
      }
    }
    imb_freerectfloatImBuf(ibuf);
    IMB_assign_float_buffer(ibuf, dst_float, IB_TAKE_OWNERSHIP);
  }

  ibuf->x = newx;
}

static void scale_up_y(ImBuf *ibuf, int newy)
{
  using namespace blender;
  uchar4 *dst_byte = nullptr;
  float *dst_float = nullptr;
  alloc_scale_dst_buffers(ibuf, ibuf->x, newy, &dst_byte, &dst_float);
  if (dst_byte == nullptr && dst_float == nullptr) {
    return;
  }

  const float add = (ibuf->y - 1.001f) / (newy - 1.0f);

  /* Byte pixels. */
  if (dst_byte != nullptr) {
    /* Special case: source is 1px high (see #70356). */
    if (UNLIKELY(ibuf->y == 1)) {
      for (int y = newy; y > 0; y--) {
        memcpy(dst_byte, ibuf->byte_buffer.data, sizeof(uchar4) * ibuf->x);
        dst_byte += ibuf->x;
      }
    }
    else {
      for (int x = ibuf->x; x > 0; x--) {
        float sample = 0;
        const uchar4 *src_byte = reinterpret_cast<const uchar4 *>(ibuf->byte_buffer.data) +
                                 (x - 1);
        uchar4 *dst_byte_ptr = dst_byte + (x - 1);

        float4 val = float4(src_byte[0]);
        float4 nval = float4(src_byte[ibuf->x]);
        float4 diff = nval - val;
        val += 0.5f;
        src_byte += ibuf->x * 2;

        for (int y = newy; y > 0; y--) {
          if (sample >= 1.0f) {
            sample -= 1.0f;
            val = nval;
            nval = float4(src_byte[0]);
            diff = nval - val;
            val += 0.5f;
            src_byte += ibuf->x;
          }
          float4 pix = val + sample * diff;
          *dst_byte_ptr = uchar4(pix);
          dst_byte_ptr += ibuf->x;
          sample += add;
        }
      }
    }
    imb_freerectImBuf(ibuf);
    IMB_assign_byte_buffer(ibuf, reinterpret_cast<uint8_t *>(dst_byte), IB_TAKE_OWNERSHIP);
  }

  /* Float pixels. */
  if (dst_float != nullptr) {
    /* Special case: source is 1px high (see #70356). */
    if (UNLIKELY(ibuf->y == 1)) {
      for (int y = newy; y > 0; y--) {
        memcpy(dst_float, ibuf->float_buffer.data, sizeof(float) * ibuf->x * ibuf->channels);
        dst_float += ibuf->x * ibuf->channels;
      }
    }
    else {
      for (int x = ibuf->x; x > 0; x--) {
        float sample = 0;
        const float *src_float = ibuf->float_buffer.data + ibuf->channels * (x - 1);
        float *dst_float_ptr = dst_float + ibuf->channels * (x - 1);

        float4 val = load_pixel(src_float, ibuf->channels);
        float4 nval = load_pixel(src_float + ibuf->channels * ibuf->x, ibuf->channels);
        float4 diff = nval - val;
        src_float += ibuf->channels * ibuf->x * 2;

        for (int y = newy; y > 0; y--) {
          if (sample >= 1.0f) {
            sample -= 1.0f;

            val = nval;
            nval = load_pixel(src_float, ibuf->channels);
            diff = nval - val;
            src_float += ibuf->channels * ibuf->x;
          }
          float4 pix = val + sample * diff;
          store_pixel(dst_float_ptr, pix, ibuf->channels);
          dst_float_ptr += ibuf->channels * ibuf->x;
          sample += add;
        }
      }
    }
    imb_freerectfloatImBuf(ibuf);
    IMB_assign_float_buffer(ibuf, dst_float, IB_TAKE_OWNERSHIP);
  }

  ibuf->y = newy;
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

  if (newx != 0 && (newx < ibuf->x)) {
    scale_down_x(ibuf, newx);
  }
  if (newy != 0 && (newy < ibuf->y)) {
    scale_down_y(ibuf, newy);
  }
  if (newx != 0 && (newx > ibuf->x)) {
    scale_up_x(ibuf, newx);
  }
  if (newy != 0 && (newy > ibuf->y)) {
    scale_up_y(ibuf, newy);
  }

  return true;
}

bool IMB_scalefastImBuf(ImBuf *ibuf, uint newx, uint newy)
{
  BLI_assert_msg(newx > 0 && newy > 0, "Images must be at least 1 on both dimensions!");
  if (ibuf == nullptr) {
    return false;
  }
  if (newx == ibuf->x && newy == ibuf->y) {
    return false;
  }

  /* Create destination buffers. */
  uint *dst_byte_buffer = nullptr;
  if (ibuf->byte_buffer.data) {
    dst_byte_buffer = static_cast<uint *>(
        MEM_mallocN(sizeof(uint) * newx * newy, "scale byte buffer"));
  }
  float *dst_float_buffer = nullptr;
  const int channels = ibuf->channels;
  if (ibuf->float_buffer.data) {
    dst_float_buffer = static_cast<float *>(
        MEM_mallocN(sizeof(float) * channels * newx * newy, "scale float buffer"));
  }
  if (dst_byte_buffer == nullptr && dst_float_buffer == nullptr) {
    return false;
  }

  /* Processing. Step through pixels in fixed point coordinates. */
  constexpr int FRAC_BITS = 16;
  int64_t stepx = ((int64_t(ibuf->x) << FRAC_BITS) + newx / 2) / newx;
  int64_t stepy = ((int64_t(ibuf->y) << FRAC_BITS) + newy / 2) / newy;
  if (dst_byte_buffer != nullptr) {
    uint *dst = dst_byte_buffer;
    int64_t posy = 0;
    for (int y = 0; y < newy; y++, posy += stepy) {
      const uint *src = reinterpret_cast<const uint *>(ibuf->byte_buffer.data) +
                        (posy >> FRAC_BITS) * ibuf->x;
      int64_t posx = 0;
      for (int x = 0; x < newx; x++, posx += stepx) {
        *dst = src[posx >> FRAC_BITS];
        dst++;
      }
    }
  }
  if (dst_float_buffer != nullptr) {
    float *dst = dst_float_buffer;
    int64_t posy = 0;
    for (int y = 0; y < newy; y++, posy += stepy) {
      const float *src = reinterpret_cast<const float *>(ibuf->float_buffer.data) +
                         (posy >> FRAC_BITS) * ibuf->x * channels;
      int64_t posx = 0;
      for (int x = 0; x < newx; x++, posx += stepx) {
        const float *srcpix = src + (posx >> FRAC_BITS) * channels;
        for (int c = 0; c < channels; c++) {
          *dst++ = srcpix[c];
        }
      }
    }
  }

  /* Alter the image. */
  ibuf->x = newx;
  ibuf->y = newy;

  if (ibuf->byte_buffer.data) {
    imb_freerectImBuf(ibuf);
    IMB_assign_byte_buffer(ibuf, reinterpret_cast<uint8_t *>(dst_byte_buffer), IB_TAKE_OWNERSHIP);
  }
  if (ibuf->float_buffer.data) {
    imb_freerectfloatImBuf(ibuf);
    IMB_assign_float_buffer(ibuf, dst_float_buffer, IB_TAKE_OWNERSHIP);
  }
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
          math::interpolate_bilinear_fl(
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
