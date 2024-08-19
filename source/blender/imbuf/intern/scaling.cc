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

using blender::float2;
using blender::float3;
using blender::float4;
using blender::uchar4;

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
    const ImBuf *ibuf, uint newx, uint newy, uchar4 **r_dst_byte, float **r_dst_float)
{
  *r_dst_byte = nullptr;
  if (ibuf->byte_buffer.data != nullptr) {
    *r_dst_byte = static_cast<uchar4 *>(
        MEM_mallocN(sizeof(uchar4) * newx * newy, "scale_buf_byte"));
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

static inline float4 load_pixel(const uchar4 *ptr)
{
  return float4(ptr[0]);
}
static inline float4 load_pixel(const float *ptr)
{
  return float4(ptr[0]);
}
static inline float4 load_pixel(const float2 *ptr)
{
  return float4(ptr[0]);
}
static inline float4 load_pixel(const float3 *ptr)
{
  return float4(ptr[0]);
}
static inline float4 load_pixel(const float4 *ptr)
{
  return float4(ptr[0]);
}
static inline void store_pixel(float4 pix, uchar4 *ptr)
{
  *ptr = uchar4(blender::math::round(pix));
}
static inline void store_pixel(float4 pix, float *ptr)
{
  *ptr = pix.x;
}
static inline void store_pixel(float4 pix, float2 *ptr)
{
  memcpy(ptr, &pix, sizeof(*ptr));
}
static inline void store_pixel(float4 pix, float3 *ptr)
{
  memcpy(ptr, &pix, sizeof(*ptr));
}
static inline void store_pixel(float4 pix, float4 *ptr)
{
  *ptr = pix;
}

template<typename T> static void scale_down_x(const T *src, T *dst, int ibufx, int ibufy, int newx)
{
  const float add = (ibufx - 0.01f) / newx;
  const float inv_add = 1.0f / add;

  const T *src_start = src;
  T *dst_start = dst;
  for (int y = ibufy; y > 0; y--) {
    float sample = 0.0f;
    float4 val(0.0f);

    for (int x = newx; x > 0; x--) {
      float4 nval = -val * sample;
      sample += add;
      while (sample >= 1.0f) {
        sample -= 1.0f;
        nval += load_pixel(src);
        src++;
      }

      val = load_pixel(src);
      src++;

      float4 pix = (nval + sample * val) * inv_add;
      store_pixel(pix, dst);
      dst++;

      sample -= 1.0f;
    }
  }
  BLI_assert(src - src_start == ibufx * ibufy); /* see bug #26502. */
  BLI_assert(dst - dst_start == newx * ibufy);
}

static void scale_down_x(ImBuf *ibuf, int newx)
{
  uchar4 *dst_byte = nullptr;
  float *dst_float = nullptr;
  alloc_scale_dst_buffers(ibuf, newx, ibuf->y, &dst_byte, &dst_float);
  if (dst_byte == nullptr && dst_float == nullptr) {
    return;
  }

  /* Byte pixels. */
  if (dst_byte != nullptr) {
    const uchar4 *src = (const uchar4 *)ibuf->byte_buffer.data;
    scale_down_x(src, dst_byte, ibuf->x, ibuf->y, newx);
    imb_freerectImBuf(ibuf);
    IMB_assign_byte_buffer(ibuf, reinterpret_cast<uint8_t *>(dst_byte), IB_TAKE_OWNERSHIP);
  }

  /* Float pixels. */
  if (dst_float != nullptr) {
    if (ibuf->channels == 1) {
      scale_down_x(ibuf->float_buffer.data, dst_float, ibuf->x, ibuf->y, newx);
    }
    else if (ibuf->channels == 2) {
      const float2 *src = (const float2 *)ibuf->float_buffer.data;
      scale_down_x(src, (float2 *)dst_float, ibuf->x, ibuf->y, newx);
    }
    else if (ibuf->channels == 3) {
      const float3 *src = (const float3 *)ibuf->float_buffer.data;
      scale_down_x(src, (float3 *)dst_float, ibuf->x, ibuf->y, newx);
    }
    else if (ibuf->channels == 4) {
      const float4 *src = (const float4 *)ibuf->float_buffer.data;
      scale_down_x(src, (float4 *)dst_float, ibuf->x, ibuf->y, newx);
    }
    imb_freerectfloatImBuf(ibuf);
    IMB_assign_float_buffer(ibuf, dst_float, IB_TAKE_OWNERSHIP);
  }

  ibuf->x = newx;
}

template<typename T> static void scale_down_y(const T *src, T *dst, int ibufx, int ibufy, int newy)
{
  const float add = (ibufy - 0.01f) / newy;
  const float inv_add = 1.0f / add;

  const T *src_start = src;
  T *dst_ptr = dst;
  for (int x = ibufx - 1; x >= 0; x--) {
    src = src_start + x;
    dst_ptr = dst + x;
    float sample = 0.0f;
    float4 val(0.0f);

    for (int y = newy; y > 0; y--) {
      float4 nval = -val * sample;
      sample += add;
      while (sample >= 1.0f) {
        sample -= 1.0f;
        nval += load_pixel(src);
        src += ibufx;
      }

      val = load_pixel(src);
      src += ibufx;

      float4 pix = (nval + sample * val) * inv_add;
      store_pixel(pix, dst_ptr);
      dst_ptr += ibufx;

      sample -= 1.0f;
    }
  }
  BLI_assert(src - src_start == ibufx * ibufy); /* see bug #26502. */
  BLI_assert(dst_ptr - dst == ibufx * newy);
}

static void scale_down_y(ImBuf *ibuf, int newy)
{
  uchar4 *dst_byte = nullptr;
  float *dst_float = nullptr;
  alloc_scale_dst_buffers(ibuf, ibuf->x, newy, &dst_byte, &dst_float);
  if (dst_byte == nullptr && dst_float == nullptr) {
    return;
  }

  /* Byte pixels. */
  if (dst_byte != nullptr) {
    const uchar4 *src = (const uchar4 *)ibuf->byte_buffer.data;
    scale_down_y(src, dst_byte, ibuf->x, ibuf->y, newy);
    imb_freerectImBuf(ibuf);
    IMB_assign_byte_buffer(ibuf, reinterpret_cast<uint8_t *>(dst_byte), IB_TAKE_OWNERSHIP);
  }

  /* Float pixels. */
  if (dst_float != nullptr) {
    if (ibuf->channels == 1) {
      scale_down_y(ibuf->float_buffer.data, dst_float, ibuf->x, ibuf->y, newy);
    }
    else if (ibuf->channels == 2) {
      const float2 *src = (const float2 *)ibuf->float_buffer.data;
      scale_down_y(src, (float2 *)dst_float, ibuf->x, ibuf->y, newy);
    }
    else if (ibuf->channels == 3) {
      const float3 *src = (const float3 *)ibuf->float_buffer.data;
      scale_down_y(src, (float3 *)dst_float, ibuf->x, ibuf->y, newy);
    }
    else if (ibuf->channels == 4) {
      const float4 *src = (const float4 *)ibuf->float_buffer.data;
      scale_down_y(src, (float4 *)dst_float, ibuf->x, ibuf->y, newy);
    }
    imb_freerectfloatImBuf(ibuf);
    IMB_assign_float_buffer(ibuf, dst_float, IB_TAKE_OWNERSHIP);
  }

  ibuf->y = newy;
}

template<typename T> static void scale_up_x(const T *src, T *dst, int ibufx, int ibufy, int newx)
{
  const float add = (ibufx - 0.001f) / newx;
  const T *src_start = src;
  T *dst_ptr = dst;
  /* Special case: source is 1px wide (see #70356). */
  if (UNLIKELY(ibufx == 1)) {
    for (int y = ibufy; y > 0; y--) {
      for (int x = newx; x > 0; x--) {
        *dst = *src;
        dst++;
      }
      src++;
    }
  }
  else {
    for (int y = 0; y < ibufy; y++) {
      float sample = -0.5f + add * 0.5f;
      int counter = 0;
      src = src_start + y * ibufx;
      float4 val = load_pixel(src);
      float4 nval = load_pixel(src + 1);
      float4 diff = nval - val;
      src += 2;
      counter += 2;
      for (int x = 0; x < newx; x++) {
        if (sample >= 1.0f) {
          sample -= 1.0f;
          val = nval;
          nval = load_pixel(src);
          diff = nval - val;
          if (counter + 1 < ibufx) {
            src++;
            counter++;
          }
        }
        float4 pix = val + blender::math::max(sample, 0.0f) * diff;
        store_pixel(pix, dst_ptr);
        dst_ptr++;
        sample += add;
      }
    }
  }
  BLI_assert(src - src_start <= ibufx * ibufy);
  BLI_assert(dst_ptr - dst == newx * ibufy);
}

static void scale_up_x(ImBuf *ibuf, int newx)
{
  uchar4 *dst_byte = nullptr;
  float *dst_float = nullptr;
  alloc_scale_dst_buffers(ibuf, newx, ibuf->y, &dst_byte, &dst_float);
  if (dst_byte == nullptr && dst_float == nullptr) {
    return;
  }

  /* Byte pixels. */
  if (dst_byte != nullptr) {
    const uchar4 *src = (const uchar4 *)ibuf->byte_buffer.data;
    scale_up_x(src, dst_byte, ibuf->x, ibuf->y, newx);
    imb_freerectImBuf(ibuf);
    IMB_assign_byte_buffer(ibuf, reinterpret_cast<uint8_t *>(dst_byte), IB_TAKE_OWNERSHIP);
  }

  /* Float pixels. */
  if (dst_float != nullptr) {
    if (ibuf->channels == 1) {
      scale_up_x(ibuf->float_buffer.data, dst_float, ibuf->x, ibuf->y, newx);
    }
    else if (ibuf->channels == 2) {
      const float2 *src = (const float2 *)ibuf->float_buffer.data;
      scale_up_x(src, (float2 *)dst_float, ibuf->x, ibuf->y, newx);
    }
    else if (ibuf->channels == 3) {
      const float3 *src = (const float3 *)ibuf->float_buffer.data;
      scale_up_x(src, (float3 *)dst_float, ibuf->x, ibuf->y, newx);
    }
    else if (ibuf->channels == 4) {
      const float4 *src = (const float4 *)ibuf->float_buffer.data;
      scale_up_x(src, (float4 *)dst_float, ibuf->x, ibuf->y, newx);
    }
    imb_freerectfloatImBuf(ibuf);
    IMB_assign_float_buffer(ibuf, dst_float, IB_TAKE_OWNERSHIP);
  }

  ibuf->x = newx;
}

template<typename T> static void scale_up_y(const T *src, T *dst, int ibufx, int ibufy, int newy)
{
  const float add = (ibufy - 0.001f) / newy;
  const T *src_ptr = src;
  T *dst_ptr = dst;
  /* Special case: source is 1px high (see #70356). */
  if (UNLIKELY(ibufy == 1)) {
    for (int y = newy; y > 0; y--) {
      memcpy(dst, src, sizeof(T) * ibufx);
      dst += ibufx;
    }
  }
  else {
    for (int x = 0; x < ibufx; x++) {
      float sample = -0.5f + add * 0.5f;
      int counter = 0;
      src_ptr = src + x;
      dst_ptr = dst + x;

      float4 val = load_pixel(src_ptr);
      float4 nval = load_pixel(src_ptr + ibufx);
      float4 diff = nval - val;
      src_ptr += ibufx * 2;
      counter += 2;

      for (int y = 0; y < newy; y++) {
        if (sample >= 1.0f) {
          sample -= 1.0f;
          val = nval;
          nval = load_pixel(src_ptr);
          diff = nval - val;
          if (counter + 1 < ibufy) {
            src_ptr += ibufx;
            ++counter;
          }
        }
        float4 pix = val + blender::math::max(sample, 0.0f) * diff;
        store_pixel(pix, dst_ptr);
        dst_ptr += ibufx;
        sample += add;
      }
    }
  }
  BLI_assert(src_ptr - src <= ibufx * ibufy + ibufx - 1);
  BLI_assert(dst_ptr - dst == ibufx * newy + ibufx - 1);
}

static void scale_up_y(ImBuf *ibuf, int newy)
{
  uchar4 *dst_byte = nullptr;
  float *dst_float = nullptr;
  alloc_scale_dst_buffers(ibuf, ibuf->x, newy, &dst_byte, &dst_float);
  if (dst_byte == nullptr && dst_float == nullptr) {
    return;
  }

  /* Byte pixels. */
  if (dst_byte != nullptr) {
    const uchar4 *src = (const uchar4 *)ibuf->byte_buffer.data;
    scale_up_y(src, dst_byte, ibuf->x, ibuf->y, newy);
    imb_freerectImBuf(ibuf);
    IMB_assign_byte_buffer(ibuf, reinterpret_cast<uint8_t *>(dst_byte), IB_TAKE_OWNERSHIP);
  }

  /* Float pixels. */
  if (dst_float != nullptr) {
    if (ibuf->channels == 1) {
      scale_up_y(ibuf->float_buffer.data, dst_float, ibuf->x, ibuf->y, newy);
    }
    else if (ibuf->channels == 2) {
      const float2 *src = (const float2 *)ibuf->float_buffer.data;
      scale_up_y(src, (float2 *)dst_float, ibuf->x, ibuf->y, newy);
    }
    else if (ibuf->channels == 3) {
      const float3 *src = (const float3 *)ibuf->float_buffer.data;
      scale_up_y(src, (float3 *)dst_float, ibuf->x, ibuf->y, newy);
    }
    else if (ibuf->channels == 4) {
      const float4 *src = (const float4 *)ibuf->float_buffer.data;
      scale_up_y(src, (float4 *)dst_float, ibuf->x, ibuf->y, newy);
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

template<typename T>
static void scale_nearest(const T *src, T *dst, int ibufx, int ibufy, int newx, int newy)
{
  /* Nearest sample scaling. Step through pixels in fixed point coordinates. */
  constexpr int FRAC_BITS = 16;
  int64_t stepx = ((int64_t(ibufx) << FRAC_BITS) + newx / 2) / newx;
  int64_t stepy = ((int64_t(ibufy) << FRAC_BITS) + newy / 2) / newy;
  int64_t posy = 0;
  for (int y = 0; y < newy; y++, posy += stepy) {
    const T *row = src + (posy >> FRAC_BITS) * ibufx;
    int64_t posx = 0;
    for (int x = 0; x < newx; x++, posx += stepx) {
      *dst = row[posx >> FRAC_BITS];
      dst++;
    }
  }
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

  uchar4 *dst_byte = nullptr;
  float *dst_float = nullptr;
  alloc_scale_dst_buffers(ibuf, newx, newy, &dst_byte, &dst_float);
  if (dst_byte == nullptr && dst_float == nullptr) {
    return false;
  }

  /* Byte pixels. */
  if (dst_byte != nullptr) {
    const uchar4 *src = (const uchar4 *)ibuf->byte_buffer.data;
    scale_nearest(src, dst_byte, ibuf->x, ibuf->y, newx, newy);
    imb_freerectImBuf(ibuf);
    IMB_assign_byte_buffer(ibuf, reinterpret_cast<uint8_t *>(dst_byte), IB_TAKE_OWNERSHIP);
  }
  /* Float pixels. */
  if (dst_float != nullptr) {
    if (ibuf->channels == 1) {
      scale_nearest(ibuf->float_buffer.data, dst_float, ibuf->x, ibuf->y, newx, newy);
    }
    else if (ibuf->channels == 2) {
      const float2 *src = (const float2 *)ibuf->float_buffer.data;
      scale_nearest(src, (float2 *)dst_float, ibuf->x, ibuf->y, newx, newy);
    }
    else if (ibuf->channels == 3) {
      const float3 *src = (const float3 *)ibuf->float_buffer.data;
      scale_nearest(src, (float3 *)dst_float, ibuf->x, ibuf->y, newx, newy);
    }
    else if (ibuf->channels == 4) {
      const float4 *src = (const float4 *)ibuf->float_buffer.data;
      scale_nearest(src, (float4 *)dst_float, ibuf->x, ibuf->y, newx, newy);
    }
    imb_freerectfloatImBuf(ibuf);
    IMB_assign_float_buffer(ibuf, dst_float, IB_TAKE_OWNERSHIP);
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

bool IMB_scale(
    ImBuf *ibuf, unsigned int newx, unsigned int newy, IMBScaleFilter filter, bool threaded)
{
  BLI_assert_msg(newx > 0 && newy > 0, "Images must be at least 1 on both dimensions!");
  if (ibuf == nullptr) {
    return false;
  }
  if (newx == ibuf->x && newy == ibuf->y) {
    return false;
  }
  if (filter == IMBScaleFilter::Nearest) {
    return IMB_scalefastImBuf(ibuf, newx, newy);  //@TODO: threaded
  }
  else if (filter == IMBScaleFilter::Bilinear) {
    IMB_scaleImBuf_threaded(ibuf, newx, newy);  //@TODO: non-threaded
  }
  else if (filter == IMBScaleFilter::Box) {
    IMB_scaleImBuf(ibuf, newx, newy);  //@TODO: threaded
  }
  else {
    BLI_assert_unreachable();
    return false;
  }
  return true;
}
