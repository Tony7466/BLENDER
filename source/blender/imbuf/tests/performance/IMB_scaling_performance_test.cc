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

static void scale_nearest(bool use_float)
{
  ImBuf *img = create_src_image(use_float);
  {
    SCOPED_TIMER("scale_nearest");
    IMB_scalefastImBuf(img, DST_LARGER_X, DST_LARGER_Y);
    IMB_scalefastImBuf(img, SRC_X, SRC_Y);
    IMB_scalefastImBuf(img, DST_SMALLER_X, DST_SMALLER_Y);
  }
  IMB_freeImBuf(img);
}

static void xform_nearest(bool use_float)
{
  ImBuf *img = create_src_image(use_float);
  {
    SCOPED_TIMER("xform_nearest");
    imb_scale_via_transform(img, DST_LARGER_X, DST_LARGER_Y, IMB_FILTER_NEAREST);
    imb_scale_via_transform(img, SRC_X, SRC_Y, IMB_FILTER_NEAREST);
    imb_scale_via_transform(img, DST_SMALLER_X, DST_SMALLER_Y, IMB_FILTER_NEAREST);
  }
  IMB_freeImBuf(img);
}

static void xform_boxfilt(bool use_float)
{
  ImBuf *img = create_src_image(use_float);
  {
    SCOPED_TIMER("xform_boxfilt");
    imb_scale_via_transform(img, DST_LARGER_X, DST_LARGER_Y, IMB_FILTER_BOX);
    imb_scale_via_transform(img, SRC_X, SRC_Y, IMB_FILTER_BOX);
    imb_scale_via_transform(img, DST_SMALLER_X, DST_SMALLER_Y, IMB_FILTER_BOX);
  }
  IMB_freeImBuf(img);
}

static void scale_bilinear_st(bool use_float)
{
  ImBuf *img = create_src_image(use_float);
  {
    SCOPED_TIMER("scale_bilin_st");
    IMB_scaleImBuf(img, DST_LARGER_X, DST_LARGER_Y);
    IMB_scaleImBuf(img, SRC_X, SRC_Y);
    IMB_scaleImBuf(img, DST_SMALLER_X, DST_SMALLER_Y);
  }
  IMB_freeImBuf(img);
}

static void scale_bilinear(bool use_float)
{
  ImBuf *img = create_src_image(use_float);
  {
    SCOPED_TIMER("scale_bilinear");
    IMB_scaleImBuf_threaded(img, DST_LARGER_X, DST_LARGER_Y);
    IMB_scaleImBuf_threaded(img, SRC_X, SRC_Y);
    IMB_scaleImBuf_threaded(img, DST_SMALLER_X, DST_SMALLER_Y);
  }
  IMB_freeImBuf(img);
}

static void xform_bilinear(bool use_float)
{
  ImBuf *img = create_src_image(use_float);
  {
    SCOPED_TIMER("xform_bilinear");
    imb_scale_via_transform(img, DST_LARGER_X, DST_LARGER_Y, IMB_FILTER_BILINEAR);
    imb_scale_via_transform(img, SRC_X, SRC_Y, IMB_FILTER_BILINEAR);
    imb_scale_via_transform(img, DST_SMALLER_X, DST_SMALLER_Y, IMB_FILTER_BILINEAR);
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
