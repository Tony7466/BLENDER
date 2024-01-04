/* SPDX-FileCopyrightText: 2006-2008 Peter Schlaile < peter [at] schlaile [dot] de >.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup spseq
 */

#include <cmath>
#include <cstring>

#include "BLI_math_vector.hh"
#include "BLI_task.h"
#include "BLI_task.hh"
#include "BLI_utildefines.h"

#include "IMB_colormanagement.h"
#include "IMB_imbuf.h"
#include "IMB_imbuf_types.h"

#include "sequencer_scopes.hh"

// #define DEBUG_TIME

#ifdef DEBUG_TIME
#  include "BLI_timeit.hh"
#endif

namespace blender::ed::seq {

SeqScopes::~SeqScopes()
{
  cleanup();
}

void SeqScopes::cleanup()
{
  if (zebra_ibuf) {
    IMB_freeImBuf(zebra_ibuf);
    zebra_ibuf = nullptr;
  }
  if (waveform_ibuf) {
    IMB_freeImBuf(waveform_ibuf);
    waveform_ibuf = nullptr;
  }
  if (sep_waveform_ibuf) {
    IMB_freeImBuf(sep_waveform_ibuf);
    sep_waveform_ibuf = nullptr;
  }
  if (vector_ibuf) {
    IMB_freeImBuf(vector_ibuf);
    vector_ibuf = nullptr;
  }
  histogram.data.reinitialize(0);
}

/* XXX(@ideasman42): why is this function better than BLI_math version?
 * only difference is it does some normalize after, need to double check on this. */
static void rgb_to_yuv_normalized(const float rgb[3], float yuv[3])
{
  yuv[0] = 0.299f * rgb[0] + 0.587f * rgb[1] + 0.114f * rgb[2];
  yuv[1] = 0.492f * (rgb[2] - yuv[0]);
  yuv[2] = 0.877f * (rgb[0] - yuv[0]);

  /* Normalize. */
  yuv[1] *= 255.0f / (122 * 2.0f);
  yuv[1] += 0.5f;

  yuv[2] *= 255.0f / (157 * 2.0f);
  yuv[2] += 0.5f;
}

static void scope_put_pixel(const uchar *table, uchar *pos)
{
  uchar newval = table[*pos];
  pos[0] = pos[1] = pos[2] = newval;
  pos[3] = 255;
}

static void scope_put_pixel_single(const uchar *table, uchar *pos, int col)
{
  uint newval = table[pos[col]];
  /* So that the separate waveforms are not just pure RGB primaries, put
   * some amount of value into the other channels too: slightly reduce it,
   * and raise to 4th power. */
  uint other = newval * 31 / 32;
  other = (other * other) >> 8;
  other = (other * other) >> 8;
  pos[0] = pos[1] = pos[2] = uchar(other);
  pos[col] = uchar(newval);
  pos[3] = 255;
}

static void init_wave_table(int height, uchar wtable[256])
{
  /* For each pixel column of the image, waveform plots the intensity values
   * with height proportional to the intensity. So depending on the height of
   * the image, different amount of pixels are expected to hit the same
   * intensity. Adjust the waveform plotting table gamma factor so that
   * the waveform has decent visibility without saturating or being too dark:
   * 0.3 gamma at height=360 and below, 0.9 gamma at height 2160 (4K) and up,
   * and interpolating between those. */
  float alpha = clamp_f(ratiof(360.0f, 2160.0f, height), 0.0f, 1.0f);
  float gamma = interpf(0.9f, 0.3f, alpha);
  for (int x = 0; x < 256; x++) {
    wtable[x] = uchar(pow((float(x) + 1.0f) / 256.0f, gamma) * 255.0f);
  }
}

static ImBuf *make_waveform_view_from_ibuf_byte(ImBuf *ibuf)
{
#ifdef DEBUG_TIME
  SCOPED_TIMER_AVERAGED(__func__);
#endif
  using namespace blender;
  int w = ibuf->x;
  int h = 256;
  ImBuf *rval = IMB_allocImBuf(w, h, 32, IB_rect);
  const uchar *src = ibuf->byte_buffer.data;
  uchar *tgt = rval->byte_buffer.data;

  uchar wtable[256];
  init_wave_table(ibuf->y, wtable);

  /* IMB_colormanagement_get_luminance_byte for each pixel is quite a lot of
   * overhead, so instead get luma coefficients as 16-bit integers. */
  float coeffs[3];
  IMB_colormanagement_get_luminance_coefficients(coeffs);
  int muls[3] = {int(coeffs[0] * 65535), int(coeffs[1] * 65535), int(coeffs[2] * 65535)};

  /* Parallel over x, since each column is easily independent from others. */
  threading::parallel_for(IndexRange(ibuf->x), 16, [&](IndexRange x_range) {
    for (int y = 0; y < ibuf->y; y++) {

      for (const int x : x_range) {
        const uchar *rgb = src + 4 * (ibuf->x * y + x);
        /* +1 is "Sree's solution" from http://stereopsis.com/doubleblend.html */
        int rgb0 = rgb[0] + 1;
        int rgb1 = rgb[1] + 1;
        int rgb2 = rgb[2] + 1;
        int luma = (rgb0 * muls[0] + rgb1 * muls[1] + rgb2 * muls[2]) >> 16;
        int luma_y = clamp_i(luma, 0, 255);
        uchar *p = tgt + 4 * (w * luma_y + x);
        scope_put_pixel(wtable, p);
      }
    }
  });

  return rval;
}

static ImBuf *make_waveform_view_from_ibuf_float(ImBuf *ibuf)
{
#ifdef DEBUG_TIME
  SCOPED_TIMER_AVERAGED(__func__);
#endif
  using namespace blender;
  int w = ibuf->x;
  int h = 256;
  ImBuf *rval = IMB_allocImBuf(w, h, 32, IB_rect);
  const float *src = ibuf->float_buffer.data;
  uchar *tgt = rval->byte_buffer.data;

  uchar wtable[256];
  init_wave_table(ibuf->y, wtable);

  /* Parallel over x, since each column is easily independent from others. */
  threading::parallel_for(IndexRange(ibuf->x), 16, [&](IndexRange x_range) {
    for (int y = 0; y < ibuf->y; y++) {

      for (const int x : x_range) {
        const float *rgb = src + 4 * (ibuf->x * y + x);
        float v = IMB_colormanagement_get_luminance(rgb);
        uchar *p = tgt;

        int iv = clamp_i(int(v * h), 0, h - 1);

        p += 4 * (w * iv + x);
        scope_put_pixel(wtable, p);
      }
    }
  });

  return rval;
}

ImBuf *make_waveform_view_from_ibuf(ImBuf *ibuf)
{
  if (ibuf->float_buffer.data) {
    return make_waveform_view_from_ibuf_float(ibuf);
  }
  return make_waveform_view_from_ibuf_byte(ibuf);
}

static ImBuf *make_sep_waveform_view_from_ibuf_byte(ImBuf *ibuf)
{
#ifdef DEBUG_TIME
  SCOPED_TIMER_AVERAGED(__func__);
#endif
  using namespace blender;
  int w = ibuf->x;
  int h = 256;
  ImBuf *rval = IMB_allocImBuf(w, h, 32, IB_rect);
  const uchar *src = ibuf->byte_buffer.data;
  uchar *tgt = rval->byte_buffer.data;
  int sw = ibuf->x / 3;

  uchar wtable[256];
  init_wave_table(ibuf->y, wtable);

  /* Parallel over x, since each column is easily independent from others. */
  threading::parallel_for(IndexRange(ibuf->x), 16, [&](IndexRange x_range) {
    for (int y = 0; y < ibuf->y; y++) {
      for (const int x : x_range) {
        int c;
        const uchar *rgb = src + 4 * (ibuf->x * y + x);
        for (c = 0; c < 3; c++) {
          uchar *p = tgt;
          p += 4 * (w * rgb[c] + c * sw + x / 3);
          scope_put_pixel_single(wtable, p, c);
        }
      }
    }
  });

  return rval;
}

static ImBuf *make_sep_waveform_view_from_ibuf_float(ImBuf *ibuf)
{
#ifdef DEBUG_TIME
  SCOPED_TIMER_AVERAGED(__func__);
#endif
  using namespace blender;
  int w = ibuf->x;
  int h = 256;
  ImBuf *rval = IMB_allocImBuf(w, h, 32, IB_rect);
  const float *src = ibuf->float_buffer.data;
  uchar *tgt = rval->byte_buffer.data;
  int sw = ibuf->x / 3;

  uchar wtable[256];
  init_wave_table(ibuf->y, wtable);

  /* Parallel over x, since each column is easily independent from others. */
  threading::parallel_for(IndexRange(ibuf->x), 16, [&](IndexRange x_range) {
    for (int y = 0; y < ibuf->y; y++) {
      for (const int x : x_range) {
        int c;
        const float *rgb = src + 4 * (ibuf->x * y + x);
        for (c = 0; c < 3; c++) {
          uchar *p = tgt;
          float v = rgb[c];
          int iv = clamp_i(int(v * h), 0, h - 1);

          p += 4 * (w * iv + c * sw + x / 3);
          scope_put_pixel_single(wtable, p, c);
        }
      }
    }
  });

  return rval;
}

ImBuf *make_sep_waveform_view_from_ibuf(ImBuf *ibuf)
{
  if (ibuf->float_buffer.data) {
    return make_sep_waveform_view_from_ibuf_float(ibuf);
  }
  return make_sep_waveform_view_from_ibuf_byte(ibuf);
}

static void draw_zebra_byte(const ImBuf *src, ImBuf *ibuf, float perc)
{
#ifdef DEBUG_TIME
  SCOPED_TIMER_AVERAGED(__func__);
#endif
  using namespace blender;
  uint limit = 255.0f * perc / 100.0f;

  threading::parallel_for(IndexRange(ibuf->y), 16, [&](IndexRange y_range) {
    const uchar *p = src->byte_buffer.data + y_range.first() * ibuf->x * 4;
    uchar *o = ibuf->byte_buffer.data + y_range.first() * ibuf->x * 4;
    for (const int y : y_range) {
      for (int x = 0; x < ibuf->x; x++) {
        uchar r = *p++;
        uchar g = *p++;
        uchar b = *p++;
        uchar a = *p++;

        if (r >= limit || g >= limit || b >= limit) {
          if (((x + y) & 0x08) != 0) {
            r = 255 - r;
            g = 255 - g;
            b = 255 - b;
          }
        }
        *o++ = r;
        *o++ = g;
        *o++ = b;
        *o++ = a;
      }
    }
  });
}

static void draw_zebra_float(ImBuf *src, ImBuf *ibuf, float perc)
{
#ifdef DEBUG_TIME
  SCOPED_TIMER_AVERAGED(__func__);
#endif
  using namespace blender;

  float limit = perc / 100.0f;

  threading::parallel_for(IndexRange(ibuf->y), 16, [&](IndexRange y_range) {
    const float *p = src->float_buffer.data + y_range.first() * ibuf->x * 4;
    uchar *o = ibuf->byte_buffer.data + y_range.first() * ibuf->x * 4;
    for (const int y : y_range) {
      for (int x = 0; x < ibuf->x; x++) {
        float pix[4];
        pix[0] = *p++;
        pix[1] = *p++;
        pix[2] = *p++;
        pix[3] = *p++;
        if (pix[0] >= limit || pix[1] >= limit || pix[2] >= limit) {
          if (((x + y) & 0x08) != 0) {
            pix[0] = -pix[0];
            pix[1] = -pix[1];
            pix[2] = -pix[2];
          }
        }
        rgba_float_to_uchar(o, pix);
        o += 4;
      }
    }
  });
}

ImBuf *make_zebra_view_from_ibuf(ImBuf *ibuf, float perc)
{
  ImBuf *new_ibuf = IMB_allocImBuf(ibuf->x, ibuf->y, 32, IB_rect);

  if (ibuf->float_buffer.data) {
    draw_zebra_float(ibuf, new_ibuf, perc);
  }
  else {
    draw_zebra_byte(ibuf, new_ibuf, perc);
  }
  return new_ibuf;
}

static int get_bin_float(float f)
{
  if (f < -0.25f) {
    return 0;
  }
  if (f >= 1.25f) {
    return 511;
  }
  return int(((f + 0.25f) / 1.5f) * 512);
}

void ScopeHistogram::calc_from_ibuf(const ImBuf *ibuf)
{
#ifdef DEBUG_TIME
  SCOPED_TIMER_AVERAGED(__func__);
#endif

  const bool is_float = ibuf->float_buffer.data != nullptr;
  const int hist_size = is_float ? 512 : 256;

  Array<uint3> counts(hist_size, uint3(0));
  data = threading::parallel_reduce(
      IndexRange(ibuf->y),
      256,
      counts,
      [&](const IndexRange y_range, const Array<uint3> &init) {
        Array<uint3> res = init;

        if (is_float) {
          /* Float images spead -0.25..+1.25 range over 512 bins. */
          for (const int y : y_range) {
            const float *src = ibuf->float_buffer.data + y * ibuf->x * 4;
            for (int x = 0; x < ibuf->x; x++) {
              res[get_bin_float(src[0])].x++;
              res[get_bin_float(src[1])].y++;
              res[get_bin_float(src[2])].z++;
              src += 4;
            }
          }
        }
        else {
          /* Byte images just use 256 histogram bins, directly indexed by value. */
          for (const int y : y_range) {
            const uchar *src = ibuf->byte_buffer.data + y * ibuf->x * 4;
            for (int x = 0; x < ibuf->x; x++) {
              res[src[0]].x++;
              res[src[1]].y++;
              res[src[2]].z++;
              src += 4;
            }
          }
        }
        return res;
      },
      [&](const Array<uint3> &a, const Array<uint3> &b) {
        BLI_assert(a.size() == b.size());
        Array<uint3> res(a.size());
        for (int i = 0; i < a.size(); i++) {
          res[i] = a[i] + b[i];
        }
        return res;
      });

  max_value = uint3(0);
  for (const uint3 &v : data) {
    max_value = math::max(max_value, v);
  }
}

static void vectorscope_put_cross(uchar r, uchar g, uchar b, uchar *tgt, int w, int h, int size)
{
  float rgb[3], yuv[3];
  uchar *p;

  rgb[0] = float(r) / 255.0f;
  rgb[1] = float(g) / 255.0f;
  rgb[2] = float(b) / 255.0f;
  rgb_to_yuv_normalized(rgb, yuv);

  p = tgt + 4 * (w * int(yuv[2] * (h - 3) + 1) + int(yuv[1] * (w - 3) + 1));

  if (r == 0 && g == 0 && b == 0) {
    r = 255;
  }

  for (int y = -size; y <= size; y++) {
    for (int x = -size; x <= size; x++) {
      uchar *q = p + 4 * (y * w + x);
      q[0] = r;
      q[1] = g;
      q[2] = b;
      q[3] = 255;
    }
  }
}

static ImBuf *make_vectorscope_view_from_ibuf_byte(ImBuf *ibuf)
{
#ifdef DEBUG_TIME
  SCOPED_TIMER_AVERAGED(__func__);
#endif
  ImBuf *rval = IMB_allocImBuf(515, 515, 32, IB_rect);
  int x, y;
  const uchar *src = ibuf->byte_buffer.data;
  uchar *tgt = rval->byte_buffer.data;
  float rgb[3], yuv[3];
  int w = 515;
  int h = 515;
  float scope_gamma = 0.2;
  uchar wtable[256];

  for (x = 0; x < 256; x++) {
    wtable[x] = uchar(pow((float(x) + 1.0f) / 256.0f, scope_gamma) * 255.0f);
  }

  for (x = 0; x < 256; x++) {
    vectorscope_put_cross(255, 0, 255 - x, tgt, w, h, 1);
    vectorscope_put_cross(255, x, 0, tgt, w, h, 1);
    vectorscope_put_cross(255 - x, 255, 0, tgt, w, h, 1);
    vectorscope_put_cross(0, 255, x, tgt, w, h, 1);
    vectorscope_put_cross(0, 255 - x, 255, tgt, w, h, 1);
    vectorscope_put_cross(x, 0, 255, tgt, w, h, 1);
  }

  for (y = 0; y < ibuf->y; y++) {
    for (x = 0; x < ibuf->x; x++) {
      const uchar *src1 = src + 4 * (ibuf->x * y + x);
      uchar *p;

      rgb[0] = float(src1[0]) / 255.0f;
      rgb[1] = float(src1[1]) / 255.0f;
      rgb[2] = float(src1[2]) / 255.0f;
      rgb_to_yuv_normalized(rgb, yuv);

      p = tgt + 4 * (w * int(yuv[2] * (h - 3) + 1) + int(yuv[1] * (w - 3) + 1));
      scope_put_pixel(wtable, (uchar *)p);
    }
  }

  vectorscope_put_cross(0, 0, 0, tgt, w, h, 3);

  return rval;
}

static ImBuf *make_vectorscope_view_from_ibuf_float(ImBuf *ibuf)
{
#ifdef DEBUG_TIME
  SCOPED_TIMER_AVERAGED(__func__);
#endif
  ImBuf *rval = IMB_allocImBuf(515, 515, 32, IB_rect);
  int x, y;
  const float *src = ibuf->float_buffer.data;
  uchar *tgt = rval->byte_buffer.data;
  float rgb[3], yuv[3];
  int w = 515;
  int h = 515;
  float scope_gamma = 0.2;
  uchar wtable[256];

  for (x = 0; x < 256; x++) {
    wtable[x] = uchar(pow((float(x) + 1.0f) / 256.0f, scope_gamma) * 255.0f);
  }

  for (x = 0; x <= 255; x++) {
    vectorscope_put_cross(255, 0, 255 - x, tgt, w, h, 1);
    vectorscope_put_cross(255, x, 0, tgt, w, h, 1);
    vectorscope_put_cross(255 - x, 255, 0, tgt, w, h, 1);
    vectorscope_put_cross(0, 255, x, tgt, w, h, 1);
    vectorscope_put_cross(0, 255 - x, 255, tgt, w, h, 1);
    vectorscope_put_cross(x, 0, 255, tgt, w, h, 1);
  }

  for (y = 0; y < ibuf->y; y++) {
    for (x = 0; x < ibuf->x; x++) {
      const float *src1 = src + 4 * (ibuf->x * y + x);
      const uchar *p;

      memcpy(rgb, src1, sizeof(float[3]));

      clamp_v3(rgb, 0.0f, 1.0f);

      rgb_to_yuv_normalized(rgb, yuv);

      p = tgt + 4 * (w * int(yuv[2] * (h - 3) + 1) + int(yuv[1] * (w - 3) + 1));
      scope_put_pixel(wtable, (uchar *)p);
    }
  }

  vectorscope_put_cross(0, 0, 0, tgt, w, h, 3);

  return rval;
}

ImBuf *make_vectorscope_view_from_ibuf(ImBuf *ibuf)
{
  if (ibuf->float_buffer.data) {
    return make_vectorscope_view_from_ibuf_float(ibuf);
  }
  return make_vectorscope_view_from_ibuf_byte(ibuf);
}

}  // namespace blender::ed::seq
