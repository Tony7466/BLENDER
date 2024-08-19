/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include "testing/testing.h"

#include "IMB_imbuf.hh"

#include "BLI_math_base.hh"
#include "BLI_math_matrix.hh"
#include "BLI_timeit.hh"

using namespace blender;

static constexpr int SRC_X = 5123;
static constexpr int SRC_Y = 4091;

static constexpr int DST_SMALLER_X = (int)(SRC_X * 0.21f);
static constexpr int DST_SMALLER_Y = (int)(SRC_Y * 0.67f);

static constexpr int DST_LARGER_X = (int)(SRC_X * 1.19f);
static constexpr int DST_LARGER_Y = (int)(SRC_Y * 2.13f);

static ImBuf *create_src_image(bool use_float)
{
  ImBuf *img = IMB_allocImBuf(SRC_X, SRC_Y, 32, use_float ? IB_rectfloat : IB_rect);
  if (use_float) {
    float *pix = img->float_buffer.data;
    for (int i = 0; i < img->x * img->y; i++) {
      pix[0] = i * 0.1f;
      pix[1] = i * 2.1f;
      pix[2] = i * 0.01f;
      pix[3] = math::mod(i * 0.03f, 2.0f);
      pix += 4;
    }
  }
  else {
    uchar *pix = img->byte_buffer.data;
    for (int i = 0; i < img->x * img->y; i++) {
      pix[0] = i & 0xFF;
      pix[1] = (i * 3) & 0xFF;
      pix[2] = (i + 12345) & 0xFF;
      pix[3] = (i / 4) & 0xFF;
      pix += 4;
    }
  }
  return img;
}

static void imb_scale_via_transform(ImBuf *&src,
                                    int width,
                                    int height,
                                    eIMBInterpolationFilterMode filter)
{
  ImBuf *dst = IMB_allocImBuf(width, height, src->planes, src->flags);
  float4x4 matrix = math::from_scale<float4x4>(
      float4(float(src->x) / dst->x, float(src->y) / dst->y, 1.0f, 1.0f));
  IMB_transform(src, dst, IMB_TRANSFORM_MODE_REGULAR, filter, matrix.ptr(), nullptr);
  IMB_freeImBuf(src);
  src = dst;
}

static void imb_xform_nearest(ImBuf *&src, int width, int height)
{
  imb_scale_via_transform(src, width, height, IMB_FILTER_NEAREST);
}
static void imb_xform_bilinear(ImBuf *&src, int width, int height)
{
  imb_scale_via_transform(src, width, height, IMB_FILTER_BILINEAR);
}
static void imb_xform_box(ImBuf *&src, int width, int height)
{
  imb_scale_via_transform(src, width, height, IMB_FILTER_BOX);
}
static void imb_scale_nearest_st(ImBuf *&src, int width, int height)
{
  IMB_scale(src, width, height, IMBScaleFilter::Nearest, false);
}
static void imb_scale_bilinear(ImBuf *&src, int width, int height)
{
  IMB_scale(src, width, height, IMBScaleFilter::Bilinear, true);
}
static void imb_scale_box_st(ImBuf *&src, int width, int height)
{
  IMB_scale(src, width, height, IMBScaleFilter::Box, false);
}


static void scale_nearest(bool use_float)
{
  ImBuf *img = create_src_image(use_float);
  {
    SCOPED_TIMER("scale_nearest");
    imb_scale_nearest_st(img, DST_LARGER_X, DST_LARGER_Y);
      imb_scale_nearest_st(img, SRC_X, SRC_Y);
      imb_scale_nearest_st(img, DST_SMALLER_X, DST_SMALLER_Y);
      imb_scale_nearest_st(img, DST_LARGER_X, DST_LARGER_Y);
  }
  IMB_freeImBuf(img);
}

static void xform_nearest(bool use_float)
{
  ImBuf *img = create_src_image(use_float);
  {
    SCOPED_TIMER("xform_nearest");
    imb_xform_nearest(img, DST_LARGER_X, DST_LARGER_Y);
    imb_xform_nearest(img, SRC_X, SRC_Y);
    imb_xform_nearest(img, DST_SMALLER_X, DST_SMALLER_Y);
    imb_xform_nearest(img, DST_LARGER_X, DST_LARGER_Y);
  }
  IMB_freeImBuf(img);
}

static void xform_boxfilt(bool use_float)
{
  ImBuf *img = create_src_image(use_float);
  {
    SCOPED_TIMER("xform_boxfilt");
    imb_xform_bilinear(img, DST_LARGER_X, DST_LARGER_Y);
    imb_xform_box(img, SRC_X, SRC_Y);
    imb_xform_box(img, DST_SMALLER_X, DST_SMALLER_Y);
    imb_xform_bilinear(img, DST_LARGER_X, DST_LARGER_Y);
  }
  IMB_freeImBuf(img);
}

static void scale_bilinear_st(bool use_float)
{
  ImBuf *img = create_src_image(use_float);
  {
    SCOPED_TIMER("scale_bilin_st");
    imb_scale_box_st(img, DST_LARGER_X, DST_LARGER_Y);
    imb_scale_box_st(img, SRC_X, SRC_Y);
    imb_scale_box_st(img, DST_SMALLER_X, DST_SMALLER_Y);
    imb_scale_box_st(img, DST_LARGER_X, DST_LARGER_Y);
  }
  IMB_freeImBuf(img);
}

static void scale_bilinear(bool use_float)
{
  ImBuf *img = create_src_image(use_float);
  {
    SCOPED_TIMER("scale_bilinear");
    imb_scale_bilinear(img, DST_LARGER_X, DST_LARGER_Y);
    imb_scale_bilinear(img, SRC_X, SRC_Y);
    imb_scale_bilinear(img, DST_SMALLER_X, DST_SMALLER_Y);
    imb_scale_bilinear(img, DST_LARGER_X, DST_LARGER_Y);
  }
  IMB_freeImBuf(img);
}

static void xform_bilinear(bool use_float)
{
  ImBuf *img = create_src_image(use_float);
  {
    SCOPED_TIMER("xform_bilinear");
    imb_xform_bilinear(img, DST_LARGER_X, DST_LARGER_Y);
    imb_xform_bilinear(img, SRC_X, SRC_Y);
    imb_xform_bilinear(img, DST_SMALLER_X, DST_SMALLER_Y);
    imb_xform_bilinear(img, DST_LARGER_X, DST_LARGER_Y);
  }
  IMB_freeImBuf(img);
}

static void test_scaling_perf(bool use_float)
{
  scale_nearest(use_float);
  xform_nearest(use_float);
  xform_boxfilt(use_float);
  scale_bilinear_st(use_float);
  scale_bilinear(use_float);
  xform_bilinear(use_float);
}

TEST(imbuf_scaling, scaling_perf_byte)
{
  test_scaling_perf(false);
}

TEST(imbuf_scaling, scaling_perf_float)
{
  test_scaling_perf(true);
}
