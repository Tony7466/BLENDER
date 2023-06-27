/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_math_base_safe.h"
#include <algorithm>

#include "UI_interface.h"
#include "UI_resources.h"

#include "node_function_util.hh"

namespace blender::nodes::node_fn_easing_cc {

NODE_STORAGE_FUNCS(NodeEasing)

static void node_easing_declare(NodeDeclarationBuilder &b)
{
  b.is_function_node();
  b.add_input<decl::Float>("Value").min(-10000.0f).max(10000.0f).is_default_link_socket();
  b.add_input<decl::Float>("Slope")
      .default_value(0.5f)
      .min(0.0f)
      .max(1.0f)
      .subtype(PROP_FACTOR)
      .make_available([](bNode &node) { node_storage(node).operation = NODE_EASING_SLOPE; });
  b.add_input<decl::Float>("Scale")
      .min(0.0f)
      .max(1.0f)
      .subtype(PROP_FACTOR)
      .make_available([](bNode &node) { node_storage(node).operation = NODE_EASING_BOUNCE; });
  b.add_input<decl::Float>("Offset")
      .default_value(0.25f)
      .min(0.0f)
      .max(1.0f)
      .subtype(PROP_FACTOR)
      .make_available([](bNode &node) { node_storage(node).operation = NODE_EASING_SLOPE; });
  b.add_input<decl::Float>("Steps").default_value(4.0f).min(0.0f).max(64.0f).make_available(
      [](bNode &node) { node_storage(node).operation = NODE_EASING_STEPS; });
  b.add_input<decl::Int>("Bounces").default_value(4).min(2).max(16).make_available(
      [](bNode &node) { node_storage(node).operation = NODE_EASING_BOUNCE; });
  b.add_input<decl::Float>("Exponent")
      .default_value(2.0f)
      .min(0.0f)
      .max(10.0f)
      .make_available([](bNode &node) { node_storage(node).operation = NODE_EASING_POWER; });
  b.add_input<decl::Float>("Overshoot")
      .default_value(1.0f)
      .min(-10000.0f)
      .max(10000.0f)
      .make_available([](bNode &node) { node_storage(node).operation = NODE_EASING_BACK; });
  b.add_input<decl::Float>("Amplitude").min(0.0f).max(10000.0f).make_available([](bNode &node) {
    node_storage(node).operation = NODE_EASING_ELASTIC;
  });
  b.add_input<decl::Float>("Period").default_value(1.0f).min(0.0f).max(10000.0f).make_available(
      [](bNode &node) { node_storage(node).operation = NODE_EASING_ELASTIC; });
  b.add_input<decl::Float>("Frequency")
      .default_value(1.0f)
      .min(-10000.0f)
      .max(10000.0f)
      .make_available([](bNode &node) { node_storage(node).operation = NODE_EASING_SAWTOOTH; });
  b.add_input<decl::Float>("Width")
      .default_value(1.0f)
      .min(0.0f)
      .max(1.0f)
      .subtype(PROP_FACTOR)
      .make_available([](bNode &node) { node_storage(node).operation = NODE_EASING_SAWTOOTH; });
  b.add_input<decl::Float>("Pulse Width")
      .default_value(0.5f)
      .min(0.0f)
      .max(1.0f)
      .subtype(PROP_FACTOR)
      .make_available([](bNode &node) { node_storage(node).operation = NODE_EASING_SQUARE; });
  b.add_input<decl::Float>("A Width")
      .default_value(0.5f)
      .min(0.0f)
      .max(1.0f)
      .subtype(PROP_FACTOR)
      .make_available(
          [](bNode &node) { node_storage(node).operation = NODE_EASING_CUBIC_BEZIER; });
  b.add_input<decl::Float>("A Height")
      .default_value(1.0f)
      .min(-2.0f)
      .max(2.0f)
      .subtype(PROP_FACTOR)
      .make_available(
          [](bNode &node) { node_storage(node).operation = NODE_EASING_CUBIC_BEZIER; });
  b.add_input<decl::Float>("B Width")
      .default_value(0.5f)
      .min(0.0f)
      .max(1.0f)
      .subtype(PROP_FACTOR)
      .make_available(
          [](bNode &node) { node_storage(node).operation = NODE_EASING_CUBIC_BEZIER; });
  b.add_input<decl::Float>("B Height")
      .min(-2.0f)
      .max(2.0f)
      .subtype(PROP_FACTOR)
      .make_available(
          [](bNode &node) { node_storage(node).operation = NODE_EASING_CUBIC_BEZIER; });
  b.add_input<decl::Bool>("Ease Out").description("Change and flip direction from In to Out");
  b.add_input<decl::Bool>("In & Out")
      .description("Use both In and Out. Reversed if Out is selected");
  b.add_input<decl::Bool>("Reverse Input").description("Invert input value");
  b.add_input<decl::Float>("Mirror")
      .min(0.0f)
      .max(1.0f)
      .subtype(PROP_FACTOR)
      .description("Mirror input value at this input position");
  b.add_output<decl::Float>("Value");
};

static void node_easing_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiItemR(layout, ptr, "operation", 0, "", ICON_NONE);
}

static void node_easing_update(bNodeTree *ntree, bNode *node)
{
  const NodeEasing *data = (NodeEasing *)node->storage;
  NodeEasingOperation operation = (NodeEasingOperation)data->operation;

  int sock = 0;

  sock++;
  bNodeSocket *sockSlope = (bNodeSocket *)BLI_findlink(&node->inputs, sock++);
  bNodeSocket *sockScale = (bNodeSocket *)BLI_findlink(&node->inputs, sock++);
  bNodeSocket *sockOffset = (bNodeSocket *)BLI_findlink(&node->inputs, sock++);
  bNodeSocket *sockSteps = (bNodeSocket *)BLI_findlink(&node->inputs, sock++);
  bNodeSocket *sockBounces = (bNodeSocket *)BLI_findlink(&node->inputs, sock++);
  bNodeSocket *sockExponent = (bNodeSocket *)BLI_findlink(&node->inputs, sock++);
  bNodeSocket *sockOvershoot = (bNodeSocket *)BLI_findlink(&node->inputs, sock++);
  bNodeSocket *sockAmplitude = (bNodeSocket *)BLI_findlink(&node->inputs, sock++);
  bNodeSocket *sockPeriod = (bNodeSocket *)BLI_findlink(&node->inputs, sock++);
  bNodeSocket *sockFrequency = (bNodeSocket *)BLI_findlink(&node->inputs, sock++);
  bNodeSocket *sockWidth = (bNodeSocket *)BLI_findlink(&node->inputs, sock++);
  bNodeSocket *sockPulseWidth = (bNodeSocket *)BLI_findlink(&node->inputs, sock++);
  bNodeSocket *sockAWidth = (bNodeSocket *)BLI_findlink(&node->inputs, sock++);
  bNodeSocket *sockAHeight = (bNodeSocket *)BLI_findlink(&node->inputs, sock++);
  bNodeSocket *sockBWidth = (bNodeSocket *)BLI_findlink(&node->inputs, sock++);
  bNodeSocket *sockBHeight = (bNodeSocket *)BLI_findlink(&node->inputs, sock++);
  bNodeSocket *sockEaseOut = (bNodeSocket *)BLI_findlink(&node->inputs, sock++);
  bNodeSocket *sockEaseInAndOut = (bNodeSocket *)BLI_findlink(&node->inputs, sock++);

  const bool use_slope = ELEM(operation, NODE_EASING_SLOPE, NODE_EASING_BIAS, NODE_EASING_GAIN);
  const bool use_offset = ELEM(operation, NODE_EASING_SLOPE, NODE_EASING_DYNAMIC_CIRC);
  const bool use_handles = ELEM(operation, NODE_EASING_CUBIC_BEZIER);
  const bool use_ease_direction_controls = ELEM(operation,
                                                NODE_EASING_CIRC,
                                                NODE_EASING_CUBIC,
                                                NODE_EASING_EXPO,
                                                NODE_EASING_QUAD,
                                                NODE_EASING_QUART,
                                                NODE_EASING_QUINT,
                                                NODE_EASING_SINE,
                                                NODE_EASING_DYNAMIC_CIRC,
                                                NODE_EASING_BOUNCE,
                                                NODE_EASING_BACK,
                                                NODE_EASING_ELASTIC,
                                                NODE_EASING_POWER);
  const bool use_frequency = ELEM(operation,
                                  NODE_EASING_SAWTOOTH,
                                  NODE_EASING_TRIANGLE,
                                  NODE_EASING_SQUARE,
                                  NODE_EASING_SINUS);
  const bool use_width = ELEM(
      operation, NODE_EASING_SAWTOOTH, NODE_EASING_TRIANGLE, NODE_EASING_SINUS);
  const bool use_pulse_width = ELEM(operation, NODE_EASING_SQUARE);

  bke::nodeSetSocketAvailability(ntree, sockEaseOut, use_ease_direction_controls);
  bke::nodeSetSocketAvailability(ntree, sockEaseInAndOut, use_ease_direction_controls);
  bke::nodeSetSocketAvailability(ntree, sockSlope, use_slope);
  bke::nodeSetSocketAvailability(ntree, sockScale, operation == NODE_EASING_BOUNCE);
  bke::nodeSetSocketAvailability(ntree, sockOffset, use_offset);
  bke::nodeSetSocketAvailability(ntree, sockSteps, operation == NODE_EASING_STEPS);
  bke::nodeSetSocketAvailability(ntree, sockBounces, operation == NODE_EASING_BOUNCE);
  bke::nodeSetSocketAvailability(ntree, sockExponent, operation == NODE_EASING_POWER);
  bke::nodeSetSocketAvailability(ntree, sockOvershoot, operation == NODE_EASING_BACK);
  bke::nodeSetSocketAvailability(ntree, sockAmplitude, operation == NODE_EASING_ELASTIC);
  bke::nodeSetSocketAvailability(ntree, sockPeriod, operation == NODE_EASING_ELASTIC);
  bke::nodeSetSocketAvailability(ntree, sockFrequency, use_frequency);
  bke::nodeSetSocketAvailability(ntree, sockWidth, use_width);
  bke::nodeSetSocketAvailability(ntree, sockPulseWidth, use_pulse_width);
  bke::nodeSetSocketAvailability(ntree, sockAWidth, use_handles);
  bke::nodeSetSocketAvailability(ntree, sockAHeight, use_handles);
  bke::nodeSetSocketAvailability(ntree, sockBWidth, use_handles);
  bke::nodeSetSocketAvailability(ntree, sockBHeight, use_handles);
}

static void node_easing_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeEasing *data = MEM_cnew<NodeEasing>(__func__);
  data->operation = NODE_EASING_BOUNCE;
  node->storage = data;
}

/* Local functions. */
static int set_direction(const bool out, const bool inout)
{
  if (inout) {
    return out ? NODE_EASING_DIRECTION_OUT_IN : NODE_EASING_DIRECTION_IN_OUT;
  }
  return out ? NODE_EASING_DIRECTION_OUT : NODE_EASING_DIRECTION_IN;
}

static float clamp_range(const float value, const float a, const float b)
{
  return (a < b) ? math::clamp(value, a, b) : math::clamp(value, b, a);
}

/* Compatible with glsl and osl mod() with negative numbers. */
static float safe_mod(float a, float b)
{
  return (b != 0.0f) ? a - b * math::floor(a / b) : 0.0f;
}

static float mirror_input(const float p, const float m)
{
  if (m <= 0.0f) {
    return p;
  }
  if (m >= 1.0f) {
    return 1.0f - p;
  }

  const float mr = 1.0f - m;
  return (p < mr) ? p * (1.0f / mr) : 1.0f - (p - mr) * (1.0f / (1.0f - mr));
}

static float periodic_width(const float p, const float w)
{
  if (w <= 0.0f) {
    return 0.0f;
  }

  if (w >= 1.0f) {
    return p;
  }

  return (p < w) ? p * (1.0f / w) : 0.0f;
}

static float map(
    const float value, const float min1, const float max1, const float min2, const float max2)
{
  return min2 + (value - min1) * (max2 - min2) / (max1 - min1);
}

static float pre_map_input(const int direction, const float x)
{
  switch (direction) {
    case NODE_EASING_DIRECTION_IN: {
      return 1.0f - x;
    }
    case NODE_EASING_DIRECTION_OUT: {
      return x;
    }
    case NODE_EASING_DIRECTION_IN_OUT: {
      if (x <= 0.5f) {
        return 1.0f - x * 2.0f;
      }
      return x * 2.0f - 1.0f;
    }
    case NODE_EASING_DIRECTION_OUT_IN: {
      if (x <= 0.5f) {
        return x * 2.0f;
      }
      return 2.0f - x * 2.0f;
    }
  }
  BLI_assert_unreachable();
  return 0.0f;
}

static float post_map_output(const int direction, const float x, const float t)
{
  switch (direction) {
    case NODE_EASING_DIRECTION_IN: {
      return 1.0f - t;
    }
    case NODE_EASING_DIRECTION_OUT: {
      return t;
    }
    case NODE_EASING_DIRECTION_IN_OUT: {
      if (x <= 0.5f) {
        return (1.0f - t) * 0.5f;
      }
      return t * 0.5f + 0.5f;
    }
    case NODE_EASING_DIRECTION_OUT_IN: {
      if (x <= 0.5f) {
        return t * 0.5f;
      }
      return (1.0f - t) * 0.5f + 0.5f;
    }
  }
  BLI_assert_unreachable();
  return 0.0f;
}

/*
 * UnitBezier, ported from webkit cubic-bezier code.
 * Copyright (C) 2008 Apple Inc. All Rights Reserved.
 * Visualizer for unit bezier: https://cubic-bezier.com/
 */
class UnitBezier {
 private:
  double ax;
  double bx;
  double cx;

  double ay;
  double by;
  double cy;

 public:
  UnitBezier(double p1x, double p1y, double p2x, double p2y)
  {
    /* Calculate the polynomial coefficients.
     * Implicit first and last control points are (0, 0) and (1,1). */
    cx = 3.0 * p1x;
    bx = 3.0 * (p2x - p1x) - cx;
    ax = 1.0 - cx - bx;

    cy = 3.0 * p1y;
    by = 3.0 * (p2y - p1y) - cy;
    ay = 1.0 - cy - by;
  }

  double sample_curve_x(double t)
  {
    /* `ax t ^ 3 + bx t ^ 2 + cx t ' expanded using Horner's rule. */
    return ((ax * t + bx) * t + cx) * t;
  }

  double sample_curve_y(double t)
  {
    return ((ay * t + by) * t + cy) * t;
  }

  double sample_curve_derivative_x(double t)
  {
    return (3.0 * ax * t + 2.0 * bx) * t + cx;
  }

  /* Given an x value, find a parametric value it came from. */
  double solve_curve_x(double x, double epsilon)
  {
    double t0;
    double t1;
    double t2;
    double x2;
    double d2;
    int i;

    /* First try a few iterations of Newton's method -- normally very fast. */
    for (t2 = x, i = 0; i < 8; i++) {
      x2 = sample_curve_x(t2) - x;
      if (fabs(x2) < epsilon) {
        return t2;
      }
      d2 = sample_curve_derivative_x(t2);
      if (fabs(d2) < 1e-6)
        break;
      t2 = t2 - x2 / d2;
    }

    /* Fall back to the bisection method for reliability. */
    t0 = 0.0;
    t1 = 1.0;
    t2 = x;

    if (t2 < t0) {
      return t0;
    }
    if (t2 > t1) {
      return t1;
    }

    while (t0 < t1) {
      x2 = sample_curve_x(t2);
      if (fabs(x2 - x) < epsilon) {
        return t2;
      }
      if (x > x2) {
        t0 = t2;
      }
      else {
        t1 = t2;
      }
      t2 = (t1 - t0) * 0.5 + t0;
    }

    /* No result found. */
    return t2;
  }

  double solve(double x, double epsilon)
  {
    return sample_curve_y(solve_curve_x(x, epsilon));
  }
};

/*
 * Simplified easing functions based on BLI_easing.h.
 * The simplified functions are unit based, designed to work on an input range of [0-1].
 * Only Ease out functions are defined here.
 * E.g. easing_expo_out(float x) vs
 * BLI_easing_expo_ease_out(float time, float begin, float change, float duration)
 * Based on Robert Penner functions, BSD-3-Clause
 * Copyright 2001 Robert Penner. All rights reserved.
 */

static float easing_circ_out(const float x)
{
  float xm = x - 1.0f;
  return math::sqrt(1.0f - xm * xm);
}

static float easing_cubic_out(const float x)
{
  float xm = x - 1.0f;
  return (xm * xm * xm + 1.0f);
}

static float easing_expo_out(const float x)
{
  const float pow_min = 0.0009765625f; /* = 2^(-10) */
  const float pow_scale = 1.0f / (1.0f - pow_min);
  return (1.0f - (math::pow(2.0f, -10.0f * x) - pow_min) * pow_scale);
}

static float easing_quad_out(const float x)
{
  return -1.0f * x * (x - 2.0f);
}

static float easing_quart_out(const float x)
{
  const float xm = x - 1.0f;
  return -1.0f * (xm * xm * xm * xm - 1.0f);
}

static float easing_quint_out(const float x)
{
  const float xm = x - 1.0f;
  return (xm * xm * xm * xm * xm + 1.0f);
}

static float easing_sine_out(const float x)
{
  return math::sin(x * float(M_PI_2));
}

static float easing_elastic_out(const float t, const float amplitude, const float period)
{
  if (t == 0.0f || t == 1.0f) {
    return t;
  }

  float x = t;
  float s;
  float f = 1.0f;
  float p = period;
  float a = amplitude;

  x = -x;
  if (!p) {
    p = 0.3f;
  }

  a += 1.0f;

  if (!a || a < 1.0f) {
    s = p / 4.0f;
    a = 1.0f;
  }
  else {
    s = p / (2.0f * float(M_PI)) * math::asin(1.0f / a);
  }

  return (f * (a * math::pow(2.0f, 10.0f * x) * math::sin((x - s) * (2.0f * float(M_PI)) / p))) +
         1.0f;
}

static float easing_back_out(const float x, const float overshoot)
{
  if (x == 0.0f || x == 1.0f) {
    return x;
  }
  const float xm = x - 1.0f;
  return (xm * xm * ((overshoot + 1.0f) * xm + overshoot) + 1.0f);
}

static float easing_bounce_out(const float t, const float amplitude)
{
  float x = t;
  if (x < (1.0f / 2.75f)) {
    return (7.5625f * x * x);
  }
  const float a = amplitude * 4.0f;
  if (x < (2.0f / 2.75f)) {
    x -= (1.5f / 2.75f);
    x = ((7.5625f * x) * x + 0.75f);
    return 1.0f - ((1.0f - x) * a);
  }
  if (x < (2.5f / 2.75f)) {
    x -= (2.25f / 2.75f);
    x = ((7.5625f * x) * x + 0.9375f);
    return 1.0f - ((1.0f - x) * a);
  }
  x -= (2.625f / 2.75f);
  x = ((7.5625f * x) * x + 0.984375f);
  return 1.0f - ((1.0f - x) * a);
}

static float easing_dynamic_bounce_out(const float x, const int bounces, const float amplitude)
{
  if (x == 0.0f || x == 1.0f) {
    return x;
  }

  float t = x;
  if (bounces == 1) {
    return easing_bounce_out(t * (1.0f / 2.75f), amplitude);
  }
  if (bounces == 2) {
    return easing_bounce_out(t * (2.0f / 2.75f), amplitude);
  }
  if (bounces == 3) {
    return easing_bounce_out(t * (2.5f / 2.75f), amplitude);
  }
  if (bounces == 4) {
    return easing_bounce_out(t, amplitude);
  }

  const int extra_bounces = bounces - 4;
  const float extra = float(extra_bounces);
  const float w = (2.5f / 2.75f);
  const float mw = 1.0f - (2.5f / 2.75f);
  const float emx = (1.0f + mw * extra);

  t = t * emx;

  if (t < 1.0f) {
    return easing_bounce_out(t, amplitude);
  }
  float f = map(t, 1.0f, emx, 0.0f, 1.0f);
  float s = (extra_bounces > 1) ? math::floor((1.0f - f) * (extra)) / (extra - 1.0f) : 1.0f;
  s = map(s, 0.0f, 1.0f, 0.1, 0.95f);
  t = map(math::fract(f * extra), 0.0f, 1.0f, w, 1.0f);
  t = easing_bounce_out(t, amplitude);
  return 1.0f - (1.0f - t) * s;
}

static float easing_power_out(const float x, const float exponent)
{
  if (x == 0.0f || x == 1.0f) {
    return x;
  }

  return safe_powf(x, math::max(exponent, FLT_MIN));
}

/* Designed to match CSS cubic-bezier function. */
static float easing_cubic_bezier_out(
    const float x, const float a0, const float a1, const float b0, const float b1)
{
  if (x == 0.0f || x == 1.0f) {
    return x;
  }

  const float aw = math::clamp(a0, 0.0f, 1.0f);
  const float bw = math::clamp(b0, 0.0f, 1.0f);

  UnitBezier *bez = new UnitBezier(a0, a1, b0, b1);
  return float(bez->solve(x, FLT_EPSILON));
}

/* Schlick bias and gain. */
static float easing_bias(const float x, const float t)
{
  if (x == 0.0f || x == 1.0f) {
    return x;
  }
  return (x / ((((1.0f / t) - 2.0f) * (1.0f - x)) + 1.0f));
}

static float easing_gain(const float x, const float t)
{
  if (x == 0.0f || x == 1.0f) {
    return x;
  }
  else if (x < 0.5f) {
    return easing_bias(x * 2.0f, t) / 2.0f;
  }
  else {
    return easing_bias(x * 2.0f - 1.0f, 1.0f - t) / 2.0f + 0.5f;
  }
}

static float easing_dynamic_circ(const float x, const float t)
{
  if (x == 0.0f || x == 1.0f) {
    return x;
  }
  else if (x < t) {
    const float xt = x / t;
    return (x * t / x) * (1.0f - (math::sqrt(1.0f - xt * xt)));
  }
  else {
    const float xm = 1.0f - x;
    const float tm = 1.0f - t;
    const float xt = math::safe_divide(xm, tm);
    return 1.0f - (x * tm / x) * (1.0f - (math::sqrt(1 - xt * xt)));
  }
}

static float easing_slope_out(const float x, const float s, float offset)
{
  if (x == 0.0f || x == 1.0f) {
    return x;
  }

  float slope = 1.0f - s;

  if (slope == 0.0f) {
    return x;
  }

  if (offset == 0.0f && x <= offset) {
    return 0.0f;
  }

  slope = math::clamp(slope, 0.0f, 0.999f);
  offset = math::clamp(offset, 0.0f, 1.0f);

  const float c = math::safe_divide(2.0f, (1.0f - slope)) - 1.0f;
  if (x <= offset) {
    return math::safe_divide(math::pow(x, c), math::pow(offset, c - 1.0f));
  }
  return 1.0f - math::safe_divide(math::pow(1.0f - x, c), math::pow(1.0f - offset, c - 1.0f));
}

/* Frequency and step based functions. */

static float easing_square_wave(const float x, const float f, const float w)
{
  if (x == 0.0f || f == 0.0f) {
    return 0.0f;
  }

  const float xf = x * f - math::floor(x * f);

  if (x == 1.0f && xf == 0.0f) {
    return 1.0f;
  }

  return (xf >= w) ? 0 : 1;
}

static float easing_sinus_wave(const float x, const float f, const float w)
{
  if (x == 0.0f || f == 0.0f) {
    return 0.0f;
  }

  const float xf = periodic_width(x * f - math::floor(x * f), w);
  return 0.5f - math::cos(2.0f * float(M_PI) * xf) / 2.0f;
}

static float easing_sawtooth_wave(const float x, const float f, const float w)
{
  if (x == 0.0f || f == 0.0f) {
    return 0.0f;
  }

  const float xf = periodic_width(x * f - math::floor(x * f), w);

  if (x == 1.0f && xf == 0.0f && w == 1.0f) {
    return 1.0f;
  }

  return xf;
}

static float easing_triangle_wave(const float x, const float f, const float w)
{
  if (x == 0.0f || f == 0.0f) {
    return 0.0f;
  }

  const float xf = periodic_width(x * f - math::floor(x * f), w);
  return (xf < 0.5f) ? xf * 2.0f : 2.0f - xf * 2.0f;
}

static float easing_step_out(const float x, const float steps)
{
  if (x == 0.0f || x == 1.0f) {
    return x;
  }

  if (steps == 0.0f) {
    return 0.0f;
  }

  return math::clamp(math::safe_divide(math::floor(x * (steps + 1.0f)), steps), 0.0f, 1.0f);
}

/* Easing MultiFunctions */

class EasingSmoothstepFunction : public mf::MultiFunction {
 public:
  EasingSmoothstepFunction()
  {
    static mf::Signature signature = create_signature();
    this->set_signature(&signature);
  }

  static mf::Signature create_signature()
  {
    mf::Signature signature;
    mf::SignatureBuilder builder{"Easing Smoothstep", signature};
    builder.single_input<float>("Value");
    builder.single_input<bool>("Reverse Input");
    builder.single_input<float>("Mirror");
    builder.single_output<float>("Value");
    return signature;
  }

  void call(const IndexMask &mask, mf::Params params, mf::Context /*context*/) const override
  {
    int param = 0;
    const VArray<float> &value = params.readonly_single_input<float>(param++, "Value");
    const VArray<bool> &reverse = params.readonly_single_input<bool>(param++, "Reverse Input");
    const VArray<float> &mirror = params.readonly_single_input<float>(param++, "Mirror");
    MutableSpan<float> r_value = params.uninitialized_single_output<float>(param++, "Value");

    mask.foreach_index_optimized<int>([&](const int i) {
      float p = math::clamp(value[i], 0.0f, 1.0f);
      p = mirror_input(p, mirror[i]);
      const float pp = p * p;
      p = (3.0f * pp - 2.0f * pp * p);
      p = reverse[i] ? 1.0f - p : p;
      r_value[i] = p;
    });
  }
};

class EasingLinearFunction : public mf::MultiFunction {
 public:
  EasingLinearFunction()
  {
    static mf::Signature signature = create_signature();
    this->set_signature(&signature);
  }

  static mf::Signature create_signature()
  {
    mf::Signature signature;
    mf::SignatureBuilder builder{"Easing Linear", signature};
    builder.single_input<float>("Value");
    builder.single_input<bool>("Reverse Input");
    builder.single_input<float>("Mirror");
    builder.single_output<float>("Value");
    return signature;
  }

  void call(const IndexMask &mask, mf::Params params, mf::Context /*context*/) const override
  {
    int param = 0;
    const VArray<float> &value = params.readonly_single_input<float>(param++, "Value");
    const VArray<bool> &reverse = params.readonly_single_input<bool>(param++, "Reverse Input");
    const VArray<float> &mirror = params.readonly_single_input<float>(param++, "Mirror");
    MutableSpan<float> r_value = params.uninitialized_single_output<float>(param++, "Value");

    mask.foreach_index_optimized<int>([&](const int i) {
      float p = math::clamp(value[i], 0.0f, 1.0f);
      p = mirror_input(p, mirror[i]);
      p = reverse[i] ? 1.0f - p : p;
      r_value[i] = p;
    });
  }
};

class EasingSawtoothFunction : public mf::MultiFunction {
 public:
  EasingSawtoothFunction()
  {
    static mf::Signature signature = create_signature();
    this->set_signature(&signature);
  }

  static mf::Signature create_signature()
  {
    mf::Signature signature;
    mf::SignatureBuilder builder{"Easing Sawtooth", signature};
    builder.single_input<float>("Value");
    builder.single_input<float>("Frequency");
    builder.single_input<float>("Width");
    builder.single_input<bool>("Reverse Input");
    builder.single_input<float>("Mirror");
    builder.single_output<float>("Value");
    return signature;
  }

  void call(const IndexMask &mask, mf::Params params, mf::Context /*context*/) const override
  {
    int param = 0;
    const VArray<float> &value = params.readonly_single_input<float>(param++, "Value");
    const VArray<float> &frequency = params.readonly_single_input<float>(param++, "Frequency");
    const VArray<float> &width = params.readonly_single_input<float>(param++, "Width");
    const VArray<bool> &reverse = params.readonly_single_input<bool>(param++, "Reverse Input");
    const VArray<float> &mirror = params.readonly_single_input<float>(param++, "Mirror");
    MutableSpan<float> r_value = params.uninitialized_single_output<float>(param++, "Value");

    mask.foreach_index_optimized<int>([&](const int i) {
      float p = math::clamp(value[i], 0.0f, 1.0f);
      p = mirror_input(p, mirror[i]);
      p = reverse[i] ? 1.0f - p : p;
      const float t = easing_sawtooth_wave(p, frequency[i], width[i]);
      r_value[i] = t;
    });
  }
};

class EasingSinusFunction : public mf::MultiFunction {
 public:
  EasingSinusFunction()
  {
    static mf::Signature signature = create_signature();
    this->set_signature(&signature);
  }

  static mf::Signature create_signature()
  {
    mf::Signature signature;
    mf::SignatureBuilder builder{"Easing Sinus", signature};
    builder.single_input<float>("Value");
    builder.single_input<float>("Frequency");
    builder.single_input<float>("Width");
    builder.single_input<bool>("Reverse Input");
    builder.single_input<float>("Mirror");
    builder.single_output<float>("Value");
    return signature;
  }

  void call(const IndexMask &mask, mf::Params params, mf::Context /*context*/) const override
  {
    int param = 0;
    const VArray<float> &value = params.readonly_single_input<float>(param++, "Value");
    const VArray<float> &frequency = params.readonly_single_input<float>(param++, "Frequency");
    const VArray<float> &width = params.readonly_single_input<float>(param++, "Width");
    const VArray<bool> &reverse = params.readonly_single_input<bool>(param++, "Reverse Input");
    const VArray<float> &mirror = params.readonly_single_input<float>(param++, "Mirror");
    MutableSpan<float> r_value = params.uninitialized_single_output<float>(param++, "Value");

    mask.foreach_index_optimized<int>([&](const int i) {
      float p = math::clamp(value[i], 0.0f, 1.0f);
      p = mirror_input(p, mirror[i]);
      p = reverse[i] ? 1.0f - p : p;
      const float t = easing_sinus_wave(p, frequency[i], width[i]);
      r_value[i] = t;
    });
  }
};

class EasingSquareFunction : public mf::MultiFunction {
 public:
  EasingSquareFunction()
  {
    static mf::Signature signature = create_signature();
    this->set_signature(&signature);
  }

  static mf::Signature create_signature()
  {
    mf::Signature signature;
    mf::SignatureBuilder builder{"Easing Square", signature};
    builder.single_input<float>("Value");
    builder.single_input<float>("Frequency");
    builder.single_input<float>("Pulse Width");
    builder.single_input<bool>("Reverse Input");
    builder.single_input<float>("Mirror");
    builder.single_output<float>("Value");
    return signature;
  }

  void call(const IndexMask &mask, mf::Params params, mf::Context /*context*/) const override
  {
    int param = 0;
    const VArray<float> &value = params.readonly_single_input<float>(param++, "Value");
    const VArray<float> &frequency = params.readonly_single_input<float>(param++, "Frequency");
    const VArray<float> &width = params.readonly_single_input<float>(param++, "Pulse Width");
    const VArray<bool> &reverse = params.readonly_single_input<bool>(param++, "Reverse Input");
    const VArray<float> &mirror = params.readonly_single_input<float>(param++, "Mirror");
    MutableSpan<float> r_value = params.uninitialized_single_output<float>(param++, "Value");

    mask.foreach_index_optimized<int>([&](const int i) {
      float p = math::clamp(value[i], 0.0f, 1.0f);
      p = mirror_input(p, mirror[i]);
      p = reverse[i] ? 1.0f - p : p;
      const float t = easing_square_wave(p, frequency[i], width[i]);
      r_value[i] = t;
    });
  }
};

class EasingTriangleFunction : public mf::MultiFunction {
 public:
  EasingTriangleFunction()
  {
    static mf::Signature signature = create_signature();
    this->set_signature(&signature);
  }

  static mf::Signature create_signature()
  {
    mf::Signature signature;
    mf::SignatureBuilder builder{"Easing Triangle", signature};
    builder.single_input<float>("Value");
    builder.single_input<float>("Frequency");
    builder.single_input<float>("Width");
    builder.single_input<bool>("Reverse Input");
    builder.single_input<float>("Mirror");
    builder.single_output<float>("Value");
    return signature;
  }

  void call(const IndexMask &mask, mf::Params params, mf::Context /*context*/) const override
  {
    int param = 0;
    const VArray<float> &value = params.readonly_single_input<float>(param++, "Value");
    const VArray<float> &frequency = params.readonly_single_input<float>(param++, "Frequency");
    const VArray<float> &width = params.readonly_single_input<float>(param++, "Width");
    const VArray<bool> &reverse = params.readonly_single_input<bool>(param++, "Reverse Input");
    const VArray<float> &mirror = params.readonly_single_input<float>(param++, "Mirror");
    MutableSpan<float> r_value = params.uninitialized_single_output<float>(param++, "Value");

    mask.foreach_index_optimized<int>([&](const int i) {
      float p = math::clamp(value[i], 0.0f, 1.0f);
      p = mirror_input(p, mirror[i]);
      p = reverse[i] ? 1.0f - p : p;
      const float t = easing_triangle_wave(p, frequency[i], width[i]);
      r_value[i] = t;
    });
  }
};

class EasingStepFunction : public mf::MultiFunction {
 public:
  EasingStepFunction()
  {
    static mf::Signature signature = create_signature();
    this->set_signature(&signature);
  }

  static mf::Signature create_signature()
  {
    mf::Signature signature;
    mf::SignatureBuilder builder{"Easing Steps", signature};
    builder.single_input<float>("Value");
    builder.single_input<float>("Steps");
    builder.single_input<bool>("Reverse Input");
    builder.single_input<float>("Mirror");
    builder.single_output<float>("Value");
    return signature;
  }

  void call(const IndexMask &mask, mf::Params params, mf::Context /*context*/) const override
  {
    int param = 0;
    const VArray<float> &value = params.readonly_single_input<float>(param++, "Value");
    const VArray<float> &steps = params.readonly_single_input<float>(param++, "Steps");
    const VArray<bool> &reverse = params.readonly_single_input<bool>(param++, "Reverse Input");
    const VArray<float> &mirror = params.readonly_single_input<float>(param++, "Mirror");
    MutableSpan<float> r_value = params.uninitialized_single_output<float>(param++, "Value");

    mask.foreach_index_optimized<int>([&](const int i) {
      float p = math::clamp(value[i], 0.0f, 1.0f);
      p = mirror_input(p, mirror[i]);
      p = reverse[i] ? 1.0f - p : p;
      const float t = easing_step_out(p, steps[i]);
      r_value[i] = t;
    });
  }
};

class EasingBiasFunction : public mf::MultiFunction {

 public:
  EasingBiasFunction()
  {
    static mf::Signature signature = create_signature();
    this->set_signature(&signature);
  }

  static mf::Signature create_signature()
  {
    mf::Signature signature;
    mf::SignatureBuilder builder{"Easing Bias", signature};
    builder.single_input<float>("Value");
    builder.single_input<float>("Slope");
    builder.single_input<bool>("Reverse Input");
    builder.single_input<float>("Mirror");
    builder.single_output<float>("Value");
    return signature;
  }

  void call(const IndexMask &mask, mf::Params params, mf::Context /*context*/) const override
  {
    int param = 0;
    const VArray<float> &value = params.readonly_single_input<float>(param++, "Value");
    const VArray<float> &slope = params.readonly_single_input<float>(param++, "Slope");
    const VArray<bool> &reverse = params.readonly_single_input<bool>(param++, "Reverse Input");
    const VArray<float> &mirror = params.readonly_single_input<float>(param++, "Mirror");
    MutableSpan<float> r_value = params.uninitialized_single_output<float>(param++, "Value");

    mask.foreach_index_optimized<int>([&](const int i) {
      float p = math::clamp(value[i], 0.0f, 1.0f);
      p = mirror_input(p, mirror[i]);
      p = reverse[i] ? 1.0f - p : p;
      const float t = easing_bias(p, slope[i]);
      r_value[i] = t;
    });
  }
};

class EasingGainFunction : public mf::MultiFunction {

 public:
  EasingGainFunction()
  {
    static mf::Signature signature = create_signature();
    this->set_signature(&signature);
  }

  static mf::Signature create_signature()
  {
    mf::Signature signature;
    mf::SignatureBuilder builder{"Easing Gain", signature};
    builder.single_input<float>("Value");
    builder.single_input<float>("Slope");
    builder.single_input<bool>("Reverse Input");
    builder.single_input<float>("Mirror");
    builder.single_output<float>("Value");
    return signature;
  }

  void call(const IndexMask &mask, mf::Params params, mf::Context /*context*/) const override
  {
    int param = 0;
    const VArray<float> &value = params.readonly_single_input<float>(param++, "Value");
    const VArray<float> &slope = params.readonly_single_input<float>(param++, "Slope");
    const VArray<bool> &reverse = params.readonly_single_input<bool>(param++, "Reverse Input");
    const VArray<float> &mirror = params.readonly_single_input<float>(param++, "Mirror");
    MutableSpan<float> r_value = params.uninitialized_single_output<float>(param++, "Value");

    mask.foreach_index_optimized<int>([&](const int i) {
      float p = math::clamp(value[i], 0.0f, 1.0f);
      p = mirror_input(p, mirror[i]);
      p = reverse[i] ? 1.0f - p : p;
      const float t = easing_gain(p, slope[i]);
      r_value[i] = t;
    });
  }
};

class EasingSlopeFunction : public mf::MultiFunction {

 public:
  EasingSlopeFunction()
  {
    static mf::Signature signature = create_signature();
    this->set_signature(&signature);
  }

  static mf::Signature create_signature()
  {
    mf::Signature signature;
    mf::SignatureBuilder builder{"Easing Slope", signature};
    builder.single_input<float>("Value");
    builder.single_input<float>("Slope");
    builder.single_input<float>("Offset");
    builder.single_input<bool>("Reverse Input");
    builder.single_input<float>("Mirror");
    builder.single_output<float>("Value");
    return signature;
  }

  void call(const IndexMask &mask, mf::Params params, mf::Context /*context*/) const override
  {
    int param = 0;
    const VArray<float> &value = params.readonly_single_input<float>(param++, "Value");
    const VArray<float> &slope = params.readonly_single_input<float>(param++, "Slope");
    const VArray<float> &offset = params.readonly_single_input<float>(param++, "Offset");
    const VArray<bool> &reverse = params.readonly_single_input<bool>(param++, "Reverse Input");
    const VArray<float> &mirror = params.readonly_single_input<float>(param++, "Mirror");
    MutableSpan<float> r_value = params.uninitialized_single_output<float>(param++, "Value");

    mask.foreach_index_optimized<int>([&](const int i) {
      float p = math::clamp(value[i], 0.0f, 1.0f);
      p = mirror_input(p, mirror[i]);
      p = reverse[i] ? 1.0f - p : p;
      const float t = easing_slope_out(p, slope[i], offset[i]);
      r_value[i] = t;
    });
  }
};

class EasingDynamicCircularFunction : public mf::MultiFunction {

 public:
  EasingDynamicCircularFunction()
  {
    static mf::Signature signature = create_signature();
    this->set_signature(&signature);
  }

  static mf::Signature create_signature()
  {
    mf::Signature signature;
    mf::SignatureBuilder builder{"Easing Slope", signature};
    builder.single_input<float>("Value");
    builder.single_input<float>("Offset");
    builder.single_input<bool>("Ease Out");
    builder.single_input<bool>("In & Out");
    builder.single_input<bool>("Reverse Input");
    builder.single_input<float>("Mirror");
    builder.single_output<float>("Value");
    return signature;
  }

  void call(const IndexMask &mask, mf::Params params, mf::Context /*context*/) const override
  {
    int param = 0;
    const VArray<float> &value = params.readonly_single_input<float>(param++, "Value");
    const VArray<float> &offset = params.readonly_single_input<float>(param++, "Offset");
    const VArray<bool> &ease_out = params.readonly_single_input<bool>(param++, "Ease Out");
    const VArray<bool> &in_out = params.readonly_single_input<bool>(param++, "In & Out");
    const VArray<bool> &reverse = params.readonly_single_input<bool>(param++, "Reverse Input");
    const VArray<float> &mirror = params.readonly_single_input<float>(param++, "Mirror");
    MutableSpan<float> r_value = params.uninitialized_single_output<float>(param++, "Value");

    mask.foreach_index_optimized<int>([&](const int i) {
      const int direction = set_direction(ease_out[i], in_out[i]);
      float p = math::clamp(value[i], 0.0f, 1.0f);
      p = mirror_input(p, mirror[i]);
      p = reverse[i] ? 1.0f - p : p;
      float t = pre_map_input(direction, p);
      t = easing_dynamic_circ(t, offset[i]);
      t = post_map_output(direction, p, t);
      r_value[i] = t;
    });
  }
};

class EasingPowerFunction : public mf::MultiFunction {
 public:
  EasingPowerFunction()
  {
    static mf::Signature signature = create_signature();
    this->set_signature(&signature);
  }

  static mf::Signature create_signature()
  {
    mf::Signature signature;
    mf::SignatureBuilder builder{"Easing Power", signature};
    builder.single_input<float>("Value");
    builder.single_input<float>("Exponent");
    builder.single_input<bool>("Ease Out");
    builder.single_input<bool>("In & Out");
    builder.single_input<bool>("Reverse Input");
    builder.single_input<float>("Mirror");
    builder.single_output<float>("Value");
    return signature;
  }

  void call(const IndexMask &mask, mf::Params params, mf::Context /*context*/) const override
  {
    int param = 0;
    const VArray<float> &value = params.readonly_single_input<float>(param++, "Value");
    const VArray<float> &exponent = params.readonly_single_input<float>(param++, "Exponent");
    const VArray<bool> &ease_out = params.readonly_single_input<bool>(param++, "Ease Out");
    const VArray<bool> &in_out = params.readonly_single_input<bool>(param++, "In & Out");
    const VArray<bool> &reverse = params.readonly_single_input<bool>(param++, "Reverse Input");
    const VArray<float> &mirror = params.readonly_single_input<float>(param++, "Mirror");
    MutableSpan<float> r_value = params.uninitialized_single_output<float>(param++, "Value");

    mask.foreach_index_optimized<int>([&](const int i) {
      const int direction = set_direction(ease_out[i], in_out[i]);
      float p = math::clamp(value[i], 0.0f, 1.0f);
      p = mirror_input(p, mirror[i]);
      p = reverse[i] ? 1.0f - p : p;
      float t = pre_map_input(direction, p);
      t = easing_power_out(t, exponent[i]);
      t = post_map_output(direction, p, t);
      r_value[i] = t;
    });
  }
};

class EasingDynamicBounceFunction : public mf::MultiFunction {
 public:
  EasingDynamicBounceFunction()
  {
    static mf::Signature signature = create_signature();
    this->set_signature(&signature);
  }

  static mf::Signature create_signature()
  {
    mf::Signature signature;
    mf::SignatureBuilder builder{"Easing Bounce", signature};
    builder.single_input<float>("Value");
    builder.single_input<float>("Scale");
    builder.single_input<int>("Bounces");
    builder.single_input<bool>("Ease Out");
    builder.single_input<bool>("In & Out");
    builder.single_input<bool>("Reverse Input");
    builder.single_input<float>("Mirror");
    builder.single_output<float>("Value");
    return signature;
  }

  void call(const IndexMask &mask, mf::Params params, mf::Context /*context*/) const override
  {
    int param = 0;
    const VArray<float> &value = params.readonly_single_input<float>(param++, "Value");
    const VArray<float> &scale = params.readonly_single_input<float>(param++, "Scale");
    const VArray<int> &bounces = params.readonly_single_input<int>(param++, "Bounces");
    const VArray<bool> &ease_out = params.readonly_single_input<bool>(param++, "Ease Out");
    const VArray<bool> &in_out = params.readonly_single_input<bool>(param++, "In & Out");
    const VArray<bool> &reverse = params.readonly_single_input<bool>(param++, "Reverse Input");
    const VArray<float> &mirror = params.readonly_single_input<float>(param++, "Mirror");
    MutableSpan<float> r_value = params.uninitialized_single_output<float>(param++, "Value");

    mask.foreach_index_optimized<int>([&](const int i) {
      const int direction = set_direction(ease_out[i], in_out[i]);
      float p = math::clamp(value[i], 0.0f, 1.0f);
      p = mirror_input(p, mirror[i]);
      p = reverse[i] ? 1.0f - p : p;
      float t = pre_map_input(direction, p);
      t = easing_dynamic_bounce_out(t, bounces[i], scale[i]);
      t = post_map_output(direction, p, t);
      r_value[i] = t;
    });
  }
};

class EasingCubicBezierFunction : public mf::MultiFunction {
 public:
  EasingCubicBezierFunction()
  {
    static mf::Signature signature = create_signature();
    this->set_signature(&signature);
  }

  static mf::Signature create_signature()
  {
    mf::Signature signature;
    mf::SignatureBuilder builder{"Easing Bezier", signature};
    builder.single_input<float>("Value");
    builder.single_input<float>("A Width");
    builder.single_input<float>("A Height");
    builder.single_input<float>("B Width");
    builder.single_input<float>("B Height");
    builder.single_input<bool>("Reverse Input");
    builder.single_input<float>("Mirror");
    builder.single_output<float>("Value");
    return signature;
  }

  void call(const IndexMask &mask, mf::Params params, mf::Context /*context*/) const override
  {
    int param = 0;
    const VArray<float> &value = params.readonly_single_input<float>(param++, "Value");
    const VArray<float> &a_width = params.readonly_single_input<float>(param++, "A Width");
    const VArray<float> &a_height = params.readonly_single_input<float>(param++, "A Height");
    const VArray<float> &b_width = params.readonly_single_input<float>(param++, "B Width");
    const VArray<float> &b_height = params.readonly_single_input<float>(param++, "B Height");
    const VArray<bool> &reverse = params.readonly_single_input<bool>(param++, "Reverse Input");
    const VArray<float> &mirror = params.readonly_single_input<float>(param++, "Mirror");
    MutableSpan<float> r_value = params.uninitialized_single_output<float>(param++, "Value");

    mask.foreach_index_optimized<int>([&](const int i) {
      float p = math::clamp(value[i], 0.0f, 1.0f);
      p = mirror_input(p, mirror[i]);
      p = reverse[i] ? 1.0f - p : p;
      const float t = easing_cubic_bezier_out(p, a_width[i], a_height[i], b_width[i], b_height[i]);
      r_value[i] = t;
    });
  }
};

class EasingElasticFunction : public mf::MultiFunction {
 public:
  EasingElasticFunction()
  {
    static mf::Signature signature = create_signature();
    this->set_signature(&signature);
  }

  static mf::Signature create_signature()
  {
    mf::Signature signature;
    mf::SignatureBuilder builder{"Easing Elastic", signature};
    builder.single_input<float>("Value");
    builder.single_input<float>("Amplitude");
    builder.single_input<float>("Period");
    builder.single_input<bool>("Ease Out");
    builder.single_input<bool>("In & Out");
    builder.single_input<bool>("Reverse Input");
    builder.single_input<float>("Mirror");
    builder.single_output<float>("Value");
    return signature;
  }

  void call(const IndexMask &mask, mf::Params params, mf::Context /*context*/) const override
  {
    int param = 0;
    const VArray<float> &value = params.readonly_single_input<float>(param++, "Value");
    const VArray<float> &amplitude = params.readonly_single_input<float>(param++, "Amplitude");
    const VArray<float> &period = params.readonly_single_input<float>(param++, "Period");
    const VArray<bool> &ease_out = params.readonly_single_input<bool>(param++, "Ease Out");
    const VArray<bool> &in_out = params.readonly_single_input<bool>(param++, "In & Out");
    const VArray<bool> &reverse = params.readonly_single_input<bool>(param++, "Reverse Input");
    const VArray<float> &mirror = params.readonly_single_input<float>(param++, "Mirror");
    MutableSpan<float> r_value = params.uninitialized_single_output<float>(param++, "Value");

    mask.foreach_index_optimized<int>([&](const int i) {
      const int direction = set_direction(ease_out[i], in_out[i]);
      float p = math::clamp(value[i], 0.0f, 1.0f);
      p = mirror_input(p, mirror[i]);
      p = reverse[i] ? 1.0f - p : p;
      float t = pre_map_input(direction, p);
      t = easing_elastic_out(t, amplitude[i], period[i]);
      t = post_map_output(direction, p, t);
      r_value[i] = t;
    });
  }
};

class EasingBackFunction : public mf::MultiFunction {
 public:
  EasingBackFunction()
  {
    static mf::Signature signature = create_signature();
    this->set_signature(&signature);
  }

  static mf::Signature create_signature()
  {
    mf::Signature signature;
    mf::SignatureBuilder builder{"Easing Back", signature};
    builder.single_input<float>("Value");
    builder.single_input<float>("Overshoot");
    builder.single_input<bool>("Ease Out");
    builder.single_input<bool>("In & Out");
    builder.single_input<bool>("Reverse Input");
    builder.single_input<float>("Mirror");
    builder.single_output<float>("Value");
    return signature;
  }

  void call(const IndexMask &mask, mf::Params params, mf::Context /*context*/) const override
  {
    int param = 0;
    const VArray<float> &value = params.readonly_single_input<float>(param++, "Value");
    const VArray<float> &overshoot = params.readonly_single_input<float>(param++, "Overshoot");
    const VArray<bool> &ease_out = params.readonly_single_input<bool>(param++, "Ease Out");
    const VArray<bool> &in_out = params.readonly_single_input<bool>(param++, "In & Out");
    const VArray<bool> &reverse = params.readonly_single_input<bool>(param++, "Reverse Input");
    const VArray<float> &mirror = params.readonly_single_input<float>(param++, "Mirror");
    MutableSpan<float> r_value = params.uninitialized_single_output<float>(param++, "Value");

    mask.foreach_index_optimized<int>([&](const int i) {
      const int direction = set_direction(ease_out[i], in_out[i]);
      float p = math::clamp(value[i], 0.0f, 1.0f);
      p = mirror_input(p, mirror[i]);
      p = reverse[i] ? 1.0f - p : p;
      float t = pre_map_input(direction, p);
      t = easing_back_out(t, overshoot[i]);
      t = post_map_output(direction, p, t);
      r_value[i] = t;
    });
  }
};

class EasingFunction : public mf::MultiFunction {
 private:
  int operation_;

 public:
  EasingFunction(int operation) : operation_(operation)
  {
    static mf::Signature signature = create_signature();
    this->set_signature(&signature);
  }

  static mf::Signature create_signature()
  {
    mf::Signature signature;
    mf::SignatureBuilder builder{"Easing", signature};
    builder.single_input<float>("Value");
    builder.single_input<bool>("Ease Out");
    builder.single_input<bool>("In & Out");
    builder.single_input<bool>("Reverse Input");
    builder.single_input<float>("Mirror");
    builder.single_output<float>("Value");
    return signature;
  }

  static float easing_out(const int operation, const float x)
  {
    if (x == 0.0f || x == 1.0f) {
      return x;
    }

    switch (operation) {
      case NODE_EASING_CIRC: {
        return easing_circ_out(x);
      }
      case NODE_EASING_CUBIC: {
        return easing_cubic_out(x);
      }
      case NODE_EASING_EXPO: {
        return easing_expo_out(x);
      }
      case NODE_EASING_QUAD: {
        return easing_quad_out(x);
      }
      case NODE_EASING_QUART: {
        return easing_quart_out(x);
      }
      case NODE_EASING_QUINT: {
        return easing_quint_out(x);
      }
      case NODE_EASING_SINE: {
        return easing_sine_out(x);
      }
    }
    BLI_assert_unreachable();
    return 0.0f;
  }

  void call(const IndexMask &mask, mf::Params params, mf::Context /*context*/) const override
  {
    int param = 0;
    const VArray<float> &value = params.readonly_single_input<float>(param++, "Value");
    const VArray<bool> &ease_out = params.readonly_single_input<bool>(param++, "Ease Out");
    const VArray<bool> &in_out = params.readonly_single_input<bool>(param++, "In & Out");
    const VArray<bool> &reverse = params.readonly_single_input<bool>(param++, "Reverse Input");
    const VArray<float> &mirror = params.readonly_single_input<float>(param++, "Mirror");
    MutableSpan<float> r_value = params.uninitialized_single_output<float>(param++, "Value");

    BLI_assert(ELEM(operation_,
                    NODE_EASING_CIRC,
                    NODE_EASING_CUBIC,
                    NODE_EASING_EXPO,
                    NODE_EASING_QUAD,
                    NODE_EASING_QUART,
                    NODE_EASING_QUINT,
                    NODE_EASING_SINE));

    mask.foreach_index_optimized<int>([&](const int i) {
      const int direction = set_direction(ease_out[i], in_out[i]);
      float p = math::clamp(value[i], 0.0f, 1.0f);
      p = mirror_input(p, mirror[i]);
      p = reverse[i] ? 1.0f - p : p;
      float t = pre_map_input(direction, p);
      t = easing_out(operation_, t);
      t = post_map_output(direction, p, t);
      r_value[i] = t;
    });
  }
};

static void node_easing_build_multi_function(blender::nodes::NodeMultiFunctionBuilder &builder)
{
  const NodeEasing data = node_storage(builder.node());

  switch (NodeEasingOperation(data.operation)) {
    case NODE_EASING_BACK:
      builder.construct_and_set_matching_fn<EasingBackFunction>();
      break;
    case NODE_EASING_ELASTIC:
      builder.construct_and_set_matching_fn<EasingElasticFunction>();
      break;
    case NODE_EASING_POWER:
      builder.construct_and_set_matching_fn<EasingPowerFunction>();
      break;
    case NODE_EASING_STEPS:
      builder.construct_and_set_matching_fn<EasingStepFunction>();
      break;
    case NODE_EASING_LINEAR:
      builder.construct_and_set_matching_fn<EasingLinearFunction>();
      break;
    case NODE_EASING_SMOOTHSTEP:
      builder.construct_and_set_matching_fn<EasingSmoothstepFunction>();
      break;
    case NODE_EASING_SLOPE:
      builder.construct_and_set_matching_fn<EasingSlopeFunction>();
      break;
    case NODE_EASING_DYNAMIC_CIRC:
      builder.construct_and_set_matching_fn<EasingDynamicCircularFunction>();
      break;
    case NODE_EASING_BIAS:
      builder.construct_and_set_matching_fn<EasingBiasFunction>();
      break;
    case NODE_EASING_GAIN:
      builder.construct_and_set_matching_fn<EasingGainFunction>();
      break;
    case NODE_EASING_CUBIC_BEZIER:
      builder.construct_and_set_matching_fn<EasingCubicBezierFunction>();
      break;
    case NODE_EASING_BOUNCE:
      builder.construct_and_set_matching_fn<EasingDynamicBounceFunction>();
      break;
    case NODE_EASING_SAWTOOTH:
      builder.construct_and_set_matching_fn<EasingSawtoothFunction>();
      break;
    case NODE_EASING_TRIANGLE:
      builder.construct_and_set_matching_fn<EasingTriangleFunction>();
      break;
    case NODE_EASING_SINUS:
      builder.construct_and_set_matching_fn<EasingSinusFunction>();
      break;
    case NODE_EASING_SQUARE:
      builder.construct_and_set_matching_fn<EasingSquareFunction>();
      break;
    case NODE_EASING_CIRC:
    case NODE_EASING_CUBIC:
    case NODE_EASING_EXPO:
    case NODE_EASING_QUAD:
    case NODE_EASING_QUART:
    case NODE_EASING_QUINT:
    case NODE_EASING_SINE:
      builder.construct_and_set_matching_fn<EasingFunction>(data.operation);
      break;
    default:
      BLI_assert_unreachable();
      break;
  }
}

}  // namespace blender::nodes::node_fn_easing_cc

void register_node_type_fn_easing()
{
  namespace file_ns = blender::nodes::node_fn_easing_cc;

  static bNodeType ntype;
  fn_node_type_base(&ntype, FN_NODE_EASING, "Easing", NODE_CLASS_CONVERTER);
  ntype.declare = file_ns::node_easing_declare;
  ntype.updatefunc = file_ns::node_easing_update;
  ntype.initfunc = file_ns::node_easing_init;
  node_type_storage(&ntype, "NodeEasing", node_free_standard_storage, node_copy_standard_storage);
  ntype.build_multi_function = file_ns::node_easing_build_multi_function;
  ntype.draw_buttons = file_ns::node_easing_layout;
  nodeRegisterType(&ntype);
}
