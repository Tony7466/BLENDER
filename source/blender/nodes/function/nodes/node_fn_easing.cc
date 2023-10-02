/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "node_function_util.hh"

namespace blender::nodes::node_fn_easing_cc {

NODE_STORAGE_FUNCS(NodeEasing)

static void node_easing_declare(NodeDeclarationBuilder &b)
{
  b.is_function_node();
  b.add_input<decl::Float>("Value").min(0.0f).max(1.0f).is_default_link_socket();
  b.add_input<decl::Float>("Strength")
      .description("Controls curvature or shape of the easing")
      .default_value(0.5f)
      .min(0.0f)
      .max(1.0f)
      .subtype(PROP_FACTOR)
      .make_available([](bNode &node) { node_storage(node).operation = NODE_EASING_VARIABLE; });
  b.add_input<decl::Float>("Frequency")
      .default_value(1.0f)
      .min(-10.0f)
      .max(10.0f)
      .make_available([](bNode &node) { node_storage(node).operation = NODE_EASING_SAWTOOTH; });
  b.add_input<decl::Int>("Count").default_value(4).min(1).max(8).make_available(
      [](bNode &node) { node_storage(node).operation = NODE_EASING_BOUNCE; });
  b.add_input<decl::Bool>("Jump Start");
  b.add_input<decl::Bool>("Jump End");
  b.add_input<decl::Float>("Point 1 X")
      .default_value(0.5f)
      .min(0.0f)
      .max(1.0f)
      .subtype(PROP_FACTOR)
      .make_available(
          [](bNode &node) { node_storage(node).operation = NODE_EASING_CUBIC_BEZIER; });
  b.add_input<decl::Float>("Point 1 Y")
      .default_value(1.0f)
      .min(-2.0f)
      .max(2.0f)
      .subtype(PROP_FACTOR)
      .make_available(
          [](bNode &node) { node_storage(node).operation = NODE_EASING_CUBIC_BEZIER; });
  b.add_input<decl::Float>("Point 2 X")
      .default_value(0.5f)
      .min(0.0f)
      .max(1.0f)
      .subtype(PROP_FACTOR)
      .make_available(
          [](bNode &node) { node_storage(node).operation = NODE_EASING_CUBIC_BEZIER; });
  b.add_input<decl::Float>("Point 2 Y")
      .min(-2.0f)
      .max(2.0f)
      .subtype(PROP_FACTOR)
      .make_available(
          [](bNode &node) { node_storage(node).operation = NODE_EASING_CUBIC_BEZIER; });
  b.add_input<decl::Float>("Inflection")
      .description("Inflection point where In-Out/Out-In transition occurs")
      .default_value(1.0f)
      .min(0.0f)
      .max(1.0f)
      .subtype(PROP_FACTOR)
      .make_available([](bNode &node) { node_storage(node).operation = NODE_EASING_VARIABLE; });
  b.add_input<decl::Bool>("Invert").description("Invert easing direction from In-Out to Out-In");
  b.add_input<decl::Float>("Mirror")
      .min(0.0f)
      .max(1.0f)
      .subtype(PROP_FACTOR)
      .description("Mirror input at this position");
  b.add_output<decl::Float>("Value");
};

static void node_easing_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiItemR(layout, ptr, "operation", UI_ITEM_NONE, "", ICON_NONE);
}

static void node_easing_update(bNodeTree *ntree, bNode *node)
{
  const NodeEasing &storage = node_storage(*node);
  const NodeEasingOperation operation = static_cast<NodeEasingOperation>(storage.operation);

  int sock = 0;

  sock++;
  bNodeSocket *sockStrength = (bNodeSocket *)BLI_findlink(&node->inputs, sock++);
  bNodeSocket *sockFrequency = (bNodeSocket *)BLI_findlink(&node->inputs, sock++);
  bNodeSocket *sockCount = (bNodeSocket *)BLI_findlink(&node->inputs, sock++);
  bNodeSocket *sockJumpStart = (bNodeSocket *)BLI_findlink(&node->inputs, sock++);
  bNodeSocket *sockJumpEnd = (bNodeSocket *)BLI_findlink(&node->inputs, sock++);
  bNodeSocket *sockP1X = (bNodeSocket *)BLI_findlink(&node->inputs, sock++);
  bNodeSocket *sockP1Y = (bNodeSocket *)BLI_findlink(&node->inputs, sock++);
  bNodeSocket *sockP2X = (bNodeSocket *)BLI_findlink(&node->inputs, sock++);
  bNodeSocket *sockP2Y = (bNodeSocket *)BLI_findlink(&node->inputs, sock++);
  bNodeSocket *sockInflection = (bNodeSocket *)BLI_findlink(&node->inputs, sock++);
  bNodeSocket *sockInvert = (bNodeSocket *)BLI_findlink(&node->inputs, sock++);

  const bool use_strength = ELEM(operation,
                                 NODE_EASING_ELASTIC,
                                 NODE_EASING_BACK,
                                 NODE_EASING_VARIABLE,
                                 NODE_EASING_BOUNCE,
                                 NODE_EASING_SNAKE,
                                 NODE_EASING_BIAS,
                                 NODE_EASING_SPRING,
                                 NODE_EASING_SQUARE,
                                 NODE_EASING_CIRC);
  const bool use_inout_controls = ELEM(operation,
                                       NODE_EASING_CUBIC,
                                       NODE_EASING_QUAD,
                                       NODE_EASING_QUART,
                                       NODE_EASING_QUINT,
                                       NODE_EASING_SINE,
                                       NODE_EASING_EXPO,
                                       NODE_EASING_CIRC,
                                       NODE_EASING_SPRING,
                                       NODE_EASING_BOUNCE,
                                       NODE_EASING_SNAKE,
                                       NODE_EASING_BACK,
                                       NODE_EASING_ELASTIC,
                                       NODE_EASING_VARIABLE,
                                       NODE_EASING_BIAS);
  const bool use_frequency = ELEM(operation,
                                  NODE_EASING_ELASTIC,
                                  NODE_EASING_STEPS,
                                  NODE_EASING_SAWTOOTH,
                                  NODE_EASING_TRIANGLE,
                                  NODE_EASING_SQUARE,
                                  NODE_EASING_SINEWAVE);

  bke::nodeSetSocketAvailability(ntree, sockInflection, use_inout_controls);
  bke::nodeSetSocketAvailability(ntree, sockInvert, use_inout_controls);
  bke::nodeSetSocketAvailability(ntree, sockStrength, use_strength);
  bke::nodeSetSocketAvailability(
      ntree, sockCount, ELEM(operation, NODE_EASING_BOUNCE, NODE_EASING_SNAKE));
  bke::nodeSetSocketAvailability(ntree, sockFrequency, use_frequency);
  bke::nodeSetSocketAvailability(ntree, sockJumpStart, operation == NODE_EASING_STEPS);
  bke::nodeSetSocketAvailability(ntree, sockJumpEnd, operation == NODE_EASING_STEPS);
  bke::nodeSetSocketAvailability(
      ntree, sockP1X, ELEM(operation, NODE_EASING_CUBIC_BEZIER, NODE_EASING_LINEAR));
  bke::nodeSetSocketAvailability(
      ntree, sockP1Y, ELEM(operation, NODE_EASING_CUBIC_BEZIER, NODE_EASING_LINEAR));
  bke::nodeSetSocketAvailability(ntree, sockP2X, operation == NODE_EASING_CUBIC_BEZIER);
  bke::nodeSetSocketAvailability(ntree, sockP2Y, operation == NODE_EASING_CUBIC_BEZIER);

  switch (operation) {
    case NODE_EASING_SQUARE:
      node_sock_label(sockStrength, "Pulse Width");
      break;
    case NODE_EASING_BACK:
      node_sock_label(sockStrength, "Overshoot");
      break;
    case NODE_EASING_ELASTIC:
      node_sock_label(sockStrength, "Amplitude");
      break;
    default:
      node_sock_label_clear(sockStrength);
      break;
  }
}

static void node_easing_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeEasing *data = MEM_cnew<NodeEasing>(__func__);
  data->operation = NODE_EASING_BOUNCE;
  node->storage = data;
}

static float mirror_input(const float t, const float m)
{
  if (m <= 0.0f) {
    return t;
  }
  if (m >= 1.0f) {
    return 1.0f - t;
  }

  const float mr = 1.0f - m;
  return (t < mr) ? t * (1.0f / mr) : 1.0f - (t - mr) * (1.0f / m);
}

/* Clamp and mirror input. */
static float process_input(const float t, const float m)
{
  return mirror_input(math::clamp(t, 0.0f, 1.0f), m);
}

static float map(const float value,
                 const float from_min,
                 const float from_max,
                 const float to_min,
                 const float to_max)
{
  const float factor = safe_divide(value - from_min, from_max - from_min);
  return to_min + factor * (to_max - to_min);
}

/*
 * UnitBezier, ported from webkit cubic-bezier code UnitBezier.h.
 * https://github.com/WebKit/webkit/blob/main/Source/WebCore/platform/graphics/UnitBezier.h
 *
 * This source version has been altered and does not support input values outside [0,1].
 *
 * Copyright (C) 2008 Apple Inc. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY APPLE INC. AND ITS CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL APPLE INC. OR ITS CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

class UnitBezier {

#define CUBIC_BEZIER_SPLINE_SAMPLES 11

  static constexpr double kBezierEpsilon = 1e-7;
  static constexpr int kMaxNewtonIterations = 4;

 private:
  double ax;
  double bx;
  double cx;

  double ay;
  double by;
  double cy;

  double spline_samples[CUBIC_BEZIER_SPLINE_SAMPLES];

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

    double deltaT = 1.0 / (CUBIC_BEZIER_SPLINE_SAMPLES - 1);
    for (int i = 0; i < CUBIC_BEZIER_SPLINE_SAMPLES; i++)
      spline_samples[i] = sample_curve_x(i * deltaT);
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
    double t0 = 0.0;
    double t1 = 0.0;
    double t2 = x;
    double x2 = 0.0;
    double d2 = 0.0;
    int i = 0;

    /* Linear interpolation of spline curve for initial guess. */
    double deltaT = 1.0 / (CUBIC_BEZIER_SPLINE_SAMPLES - 1);
    for (i = 1; i < CUBIC_BEZIER_SPLINE_SAMPLES; i++) {
      if (x <= spline_samples[i]) {
        t1 = deltaT * i;
        t0 = t1 - deltaT;
        t2 = t0 +
             (t1 - t0) * (x - spline_samples[i - 1]) / (spline_samples[i] - spline_samples[i - 1]);
        break;
      }
    }

    /* Perform a few iterations of Newton's method -- normally very fast.
     * See https://en.wikipedia.org/wiki/Newton%27s_method. */
    double newtonEpsilon = math::min(kBezierEpsilon, epsilon);
    for (i = 0; i < kMaxNewtonIterations; i++) {
      x2 = sample_curve_x(t2) - x;
      if (math::abs(x2) < newtonEpsilon) {
        return t2;
      }
      d2 = sample_curve_derivative_x(t2);
      if (math::abs(d2) < newtonEpsilon)
        break;
      t2 = t2 - x2 / d2;
    }
    if (math::abs(x2) < epsilon)
      return t2;

    /* Fall back to the bisection method for reliability. */
    while (t0 < t1) {
      x2 = sample_curve_x(t2);
      if (math::abs(x2 - x) < epsilon) {
        return t2;
      }

      if (x > x2) {
        t0 = t2;
      }
      else {
        t1 = t2;
      }
      t2 = (t1 + t0) * 0.5;
    }

    /* No result found. */
    return t2;
  }

  double solve(double x, double epsilon)
  {
    return sample_curve_y(solve_curve_x(x, epsilon));
  }
};

/* Ease functions. */

/* Ease functions unless stated are based on Robert Penner functions,
 * BSD-3-Clause Copyright 2001 Robert Penner. All rights reserved. */

/* Adapted from Godot spring function, easing_equations.h with strength control.
 * When strength is 0.5f, the function returns same value as Godot function. */
static float easing_spring_in(const float t, const float s)
{
  const float dt = 1.0f - t;
  const float strength = s * 1.5f;

  return 1.0f - (math::sin(dt * float(M_PI) * 2.0f * strength *
                           (-0.3f + strength + 2.5f * dt * dt * dt)) *
                     math::pow(t, 2.2f) +
                 dt) *
                    (1.0f + (1.2f * t));
}

/* Inspired by CSS steps easing with start and end step controls. */
static float easing_steps(const float t, const float f, const bool jump_start, const bool jump_end)
{
  if (t == 0.0f || t == 1.0f) {
    return t;
  }
  const float start = jump_start ? 1.0f : 0.0f;
  const float end = jump_end ? 1.0f : 0.0f;
  return math::clamp(math::safe_divide(math::floor(t * (f + end - start) + start), f), 0.0f, 1.0f);
}

/* Inspired by CSS linear easing with breakpoint control. */
static float easing_linear(const float t, const float x, const float y)
{
  if (t <= 0.0f) {
    return 0.0f;
  }
  if (t >= 1.0f) {
    return 1.0f;
  }
  const float cx = math::clamp(x, 0.0f, 1.0f);
  if (cx == y) {
    return t;
  }
  if (t < cx) {
    return map(t, 0.0f, cx, 0.0f, y);
  }
  else {
    return map(t, cx, 1.0f, y, 1.0f);
  }
}

/* CSS cubic-bezier easing function. Arguments match CSS order.
 * Visualizer for unit bezier here: https://cubic-bezier.com/ */
static float easing_cubic_bezier(
    const float t, const float p1x, const float p1y, const float p2x, const float p2y)
{
  if (t == 0.0f || t == 1.0f) {
    return t;
  }

  const float cp1x = math::clamp(p1x, 0.0f, 1.0f);
  const float cp2x = math::clamp(p2x, 0.0f, 1.0f);

  UnitBezier *bez = new UnitBezier(cp1x, p1y, cp2x, p2y);
  return float(bez->solve(t, FLT_EPSILON));
}

/* Schlick bias and gain. */
static float easing_bias(const float t, const float s)
{
  return t / ((((1.0f / math::clamp(s, 0.0f, 1.0f)) - 2.0f) * (1.0f - t)) + 1.0f);
}

/* Variable power curve with exponent mapped to [0-1] range.
 * When s=0 (linear), s=0.5 (smoothstep), s=1.0 (step).*/
static float easing_variable(const float t, const float s)
{
  if (s <= 0.0f) {
    return t;
  }
  if (s >= 1.0f) {
    return 0.0f;
  }
  return math::pow(t, math::safe_divide(2.0f, (1.0f - s)) - 1.0f);
}

/* Adapted from Animation Nodes. Copyright(C) 2017 Jacques Lucke.
/* Flip_bounce alternaties heights pos/neg to create a snake like curve.
 * Output matches classic Penner when flip=false, s=0.5 and bounces=4. */
static float easing_bounce_in(const float t,
                              const float s,
                              const int bounces,
                              const bool flip_bounce)
{
  float dt = 1.0f - t;

  static constexpr int MAX_BOUNCE = 8;
  /* Precomputed cab and heights: c=a/b, a=2^(bounce-1), b=2^bounce-s^(bounce-2)-1, heights=1/4^i*/
  static constexpr float cab[MAX_BOUNCE] = {
      2.0f, 1.0f, 0.8f, 0.72727f, 0.69565f, 0.68085f, 0.67368f, 0.67016f};
  static constexpr float heights[MAX_BOUNCE] = {
      1.0f, 0.25f, 0.0625f, 0.01563f, 0.00391f, 0.00098f, 0.00024f, 0.00006f};
  float widths[MAX_BOUNCE] = {0.0f};

  const int bounce_num = math::clamp(bounces, 1, MAX_BOUNCE);

  for (int i = 0; i < bounce_num; i++) {
    widths[i] = cab[bounce_num - 1] / float(math::pow(2, i));
  };

  dt += widths[0] / 2.0f;
  float width = 0.0f;
  float height = 1.0f;
  for (int i = 0; i < bounce_num; i++) {
    width = widths[i];
    if (dt <= width) {
      dt /= width;
      if (i == 0 || s >= 1.0f) {
        height = 1.0f;
      }
      else if (s <= 0.0f) {
        height = 0.0f;
      }
      else {
        if (s > 0.5f) {
          height = 1.0f / math::pow(s, float(i) * -2.0f);
        }
        else {
          height = heights[i] * s * 2.0f;
        }
      }
      if (flip_bounce && math::mod(i, 2) != 0) {
        height = -height;
      }
      break;
    }
    dt -= width;
  }
  const float z = 4.0f / width * height * dt;
  return 1.0f - (1.0f - (z - z * dt) * width);
}

static float easing_elastic_in(const float t, const float strength, const float frequency)
{
  if (strength <= 0.0f) {
    return 0.0f;
  }
  const float dt = 1.0f - t;
  const float amp = math::max(strength, FLT_MIN);

  float s;
  float fac = 1.0f;
  const float period = 1.0f / math::max(math::abs(frequency), FLT_MIN) * signf(frequency);

  if (amp < 1.0f) {
    /* Elastic_blend to prevent sharp falloff -- see elastic_blend easing.c */
    s = 0.25f * period;
    const float ts = math::abs(s);
    fac *= amp;

    if (dt < ts) {
      const float l = dt / ts;
      fac = (fac * l) + (1.0f - l);
    }
  }
  else {
    s = period / (2.0f * float(M_PI)) * math::asin(1.0f / amp);
  }
  return (-fac * (amp * math::pow(2.0f, -10.0f * dt) *
                  math::sin((dt - s) * (2.0f * float(M_PI)) * frequency)));
}

static float easing_back_in(const float t, const float overshoot)
{
  return 1.0f * t * t * ((overshoot + 1.0f) * t - overshoot);
}

static float easing_circ(const float t, const float s)
{
  if (t < s) {
    return 0.0f;
  }
  if (t >= 1.0f) {
    return 1.0f;
  }
  const float r = 1.0f - s;
  const float ts = t - s;
  return 1.0f - math::sqrt(r * r - ts * ts) - s;
}

/* Ease wrapper functions. */

/* Simplified ease_in functions are unit based with an input range of [0-1]. */
static float ease_in(const NodeEasingOperation operation,
                     const float t,
                     const float strength,
                     const float frequency,
                     const int count)
{
  if (t == 0.0f || t == 1.0f) {
    return t;
  }

  switch (operation) {
    case NODE_EASING_EXPO: {
      const float pow_min = 0.0009765625f; /* = 2^(-10) */
      const float pow_scale = 1.0f / (1.0f - pow_min);
      return (math::pow(2.0f, 10.0f * (t - 1.0f)) - pow_min) * pow_scale;
    }
    case NODE_EASING_QUAD: {
      return t * t;
    }
    case NODE_EASING_CUBIC: {
      return t * t * t;
    }
    case NODE_EASING_QUART: {
      return t * t * t * t;
    }
    case NODE_EASING_QUINT: {
      return t * t * t * t * t;
    }
    case NODE_EASING_SINE: {
      return -math::cos(t * float(M_PI_2)) + 1.0f;
    }
    case NODE_EASING_CIRC: {
      return easing_circ(t, strength);
    }
    case NODE_EASING_BIAS: {
      return easing_bias(t, strength);
    }
    case NODE_EASING_SPRING: {
      return easing_spring_in(t, strength);
    }
    case NODE_EASING_BACK: {
      const float ui_fac = 5.0f;
      return easing_back_in(t, strength * ui_fac);
    }
    case NODE_EASING_BOUNCE: {
      return easing_bounce_in(t, strength, count, false);
    }
    case NODE_EASING_SNAKE: {
      return easing_bounce_in(t, strength, count, true);
    }
    case NODE_EASING_VARIABLE: {
      return easing_variable(t, strength);
    }
    case NODE_EASING_ELASTIC: {
      const float ui_fac = 2.0f;
      return easing_elastic_in(t, strength * ui_fac, frequency);
    }
    default:
      BLI_assert_unreachable();
      return 0.0f;
  }
}

/* Ease_out can be obtained by flipping the ease_in calculations.*/
static float ease_out(const NodeEasingOperation operation,
                      const float t,
                      const float strength,
                      const float frequency,
                      const int count)
{
  return 1.0f - ease_in(operation, 1.0f - t, strength, frequency, count);
}

static float ease_inout(const NodeEasingOperation operation,
                        const float t,
                        const float strength,
                        const float frequency,
                        const int count,
                        const float inflection)
{
  const float pos = math::clamp(inflection, 0.0f, 1.0f);
  if (t < pos) {
    const float dt = map(t, 0.0f, pos, 0.0f, 1.0f);
    return ease_in(operation, dt, strength, frequency, count) * pos;
  }
  else {
    const float dt = map(t, pos, 1.0f, 0.0f, 1.0f);
    return ease_out(operation, dt, strength, frequency, count) * (1.0f - pos) + pos;
  }
}

static float ease_outin(const NodeEasingOperation operation,
                        const float t,
                        const float strength,
                        const float frequency,
                        const int count,
                        const float inflection)
{
  const float pos = math::clamp(inflection, 0.0f, 1.0f);
  if (t < pos) {
    const float dt = map(t, 0.0f, pos, 0.0f, 1.0f);
    return ease_out(operation, dt, strength, frequency, count) * pos;
  }
  else {
    const float dt = map(t, pos, 1.0f, 0.0f, 1.0f);
    return ease_in(operation, dt, strength, frequency, count) * (1.0f - pos) + pos;
  }
}

/* Periodic functions. */
static float node_periodic(const NodeEasingOperation operation,
                           const float value,
                           const float frequency,
                           const float width,
                           const float mirror)
{
  float tf = process_input(value, mirror);

  if (tf <= 0.0f || frequency == 0.0f) {
    return 0.0f;
  }

  tf = tf * frequency - math::floor(tf * frequency);

  switch (operation) {
    case NODE_EASING_SINEWAVE: {
      return 0.5f - math::cos(2.0f * float(M_PI) * tf) / 2.0f;
    }
    case NODE_EASING_TRIANGLE: {
      return (tf < 0.5f) ? tf * 2.0f : 2.0f - tf * 2.0f;
    }
    case NODE_EASING_SAWTOOTH: {
      if (value >= 1.0f && tf == 0.0f) {
        return 1.0f;
      }
      return tf;
    }
    case NODE_EASING_SQUARE: {
      if (value >= 1.0f && tf == 0.0f) {
        return 1.0f;
      }
      return (tf >= width) ? 0.0f : 1.0f;
    }
    default:
      BLI_assert_unreachable();
      return 0.0f;
  }
}

static float node_easing(const NodeEasingOperation operation,
                         const float t,
                         const float strength,
                         const float frequency,
                         const int count,
                         const float inflection,
                         const bool invert,
                         const float mirror)
{
  float dt = process_input(t, mirror);
  /* Early out for outer edges. */
  if (dt <= 0.0f) {
    return 0.0f;
  }
  else if (dt >= 1.0f) {
    return 1.0f;
  }

  return invert ? ease_outin(operation, dt, strength, frequency, count, inflection) :
                  ease_inout(operation, dt, strength, frequency, count, inflection);
}

static auto build_easing(const char *fn_name, const NodeEasingOperation operation)
{
  return mf::build::SI4_SO<float, float, bool, float, float>(
      fn_name,
      [operation](float t, float inflection, bool invert, float mirror) -> float {
        return node_easing(operation, t, 0.0f, 0.0f, 0, inflection, invert, mirror);
      },
      mf::build::exec_presets::SomeSpanOrSingle<0>());
}

static auto build_easing_with_strength(const char *fn_name, const NodeEasingOperation operation)
{
  return mf::build::SI5_SO<float, float, float, bool, float, float>(
      fn_name,
      [operation](float t, float strength, float inflection, bool invert, float mirror) -> float {
        return node_easing(operation, t, strength, 0.0f, 0, inflection, invert, mirror);
      },
      mf::build::exec_presets::SomeSpanOrSingle<0>());
}

static auto build_periodic(const char *fn_name, const NodeEasingOperation operation)
{
  return mf::build::SI3_SO<float, float, float, float>(
      fn_name,
      [operation](float t, float frequency, float mirror) -> float {
        return node_periodic(operation, t, frequency, 0.0f, mirror);
      },
      mf::build::exec_presets::SomeSpanOrSingle<0>());
}

static auto build_easing_with_count(const char *fn_name, const NodeEasingOperation operation)
{
  return mf::build::SI6_SO<float, float, int, float, bool, float, float>(
      fn_name,
      [operation](float t, float strength, int count, float inflection, bool invert, float mirror)
          -> float {
        return node_easing(operation, t, strength, 0.0f, count, inflection, invert, mirror);
      },
      mf::build::exec_presets::SomeSpanOrSingle<0>());
}

static void node_easing_build_multi_function(blender::nodes::NodeMultiFunctionBuilder &builder)
{
  const NodeEasing &storage = node_storage(builder.node());
  const NodeEasingOperation operation = NodeEasingOperation(storage.operation);
  static auto exec_preset = mf::build::exec_presets::SomeSpanOrSingle<0>();

  switch (operation) {
    case NODE_EASING_CUBIC: {
      static auto fn = build_easing("Easing Cubic", operation);
      builder.set_matching_fn(fn);
      break;
    }
    case NODE_EASING_EXPO: {
      static auto fn = build_easing("Easing Expo", operation);
      builder.set_matching_fn(fn);
      break;
    }
    case NODE_EASING_QUINT: {
      static auto fn = build_easing("Easing Quint", operation);
      builder.set_matching_fn(fn);
      break;
    }
    case NODE_EASING_SINE: {
      static auto fn = build_easing("Easing Sine", operation);
      builder.set_matching_fn(fn);
      break;
    }
    case NODE_EASING_QUAD: {
      static auto fn = build_easing("Easing Quad", operation);
      builder.set_matching_fn(fn);
      break;
    }
    case NODE_EASING_QUART: {
      static auto fn = build_easing("Easing Quart", operation);
      builder.set_matching_fn(fn);
      break;
    }
    case NODE_EASING_CIRC: {
      static auto fn = build_easing_with_strength("Easing Circ", operation);
      builder.set_matching_fn(fn);
      break;
    }
    case NODE_EASING_SPRING: {
      static auto fn = build_easing_with_strength("Easing Spring", operation);
      builder.set_matching_fn(fn);
      break;
    }
    case NODE_EASING_VARIABLE: {
      static auto fn = build_easing_with_strength("Easing Variable", operation);
      builder.set_matching_fn(fn);
      break;
    }
    case NODE_EASING_SAWTOOTH: {
      static auto fn = build_periodic("Easing Saw", operation);
      builder.set_matching_fn(fn);
      break;
    }
    case NODE_EASING_TRIANGLE: {
      static auto fn = build_periodic("Easing Triangle", operation);
      builder.set_matching_fn(fn);
      break;
    }
    case NODE_EASING_SINEWAVE: {
      static auto fn = build_periodic("Easing Sine", operation);
      builder.set_matching_fn(fn);
      break;
    }
    case NODE_EASING_BIAS: {
      static auto fn = build_easing_with_strength("Variable Bias", operation);
      builder.set_matching_fn(fn);
      break;
    }
    case NODE_EASING_BACK: {
      static auto fn = build_easing_with_strength("Easing Overshoot", operation);
      builder.set_matching_fn(fn);
      break;
    }
    case NODE_EASING_BOUNCE: {
      static auto fn = build_easing_with_count("Easing Bounce", operation);
      builder.set_matching_fn(fn);
      break;
    }
    case NODE_EASING_SNAKE: {
      static auto fn = build_easing_with_count("Easing Snake", operation);
      builder.set_matching_fn(fn);
      break;
    }
    case NODE_EASING_SQUARE: {
      static auto fn = mf::build::SI4_SO<float, float, float, float, float>(
          "Easing Square",
          [operation](float t, float strength, float frequency, float mirror) -> float {
            return node_periodic(operation, t, frequency, strength, mirror);
          },
          exec_preset);
      builder.set_matching_fn(fn);
      break;
    }
    case NODE_EASING_ELASTIC: {
      static auto fn = mf::build::SI6_SO<float, float, float, float, bool, float, float>(
          "Easing Elastic",
          [operation](float t,
                      float strength,
                      float frequency,
                      float inflection,
                      bool invert,
                      float mirror) -> float {
            return node_easing(operation, t, strength, frequency, 0, inflection, invert, mirror);
          },
          exec_preset);
      builder.set_matching_fn(fn);
      break;
    }
    case NODE_EASING_STEPS: {
      static auto fn = mf::build::SI5_SO<float, float, bool, bool, float, float>(
          "Easing Steps",
          [operation](
              float t, float frequency, bool jump_start, bool jump_end, float mirror) -> float {
            const float p = process_input(t, mirror);
            return easing_steps(p, frequency, jump_start, jump_end);
          },
          exec_preset);
      builder.set_matching_fn(fn);
      break;
    }
    case NODE_EASING_LINEAR: {
      static auto fn = mf::build::SI4_SO<float, float, float, float, float>(
          "Easing Linear",
          [operation](float t, float p1, float p2, float mirror) -> float {
            const float p = process_input(t, mirror);
            return easing_linear(p, p1, p2);
          },
          exec_preset);
      builder.set_matching_fn(fn);
      break;
    }
    case NODE_EASING_CUBIC_BEZIER: {
      static auto fn = mf::build::SI6_SO<float, float, float, float, float, float, float>(
          "Easing Cubic Bezier",
          [operation](float value, float ax, float ay, float bx, float by, float mirror) -> float {
            const float p = process_input(value, mirror);
            return easing_cubic_bezier(p, ax, ay, bx, by);
          },
          exec_preset);
      builder.set_matching_fn(fn);
      break;
    }
    default:
      BLI_assert_unreachable();
      break;
  }
}

static void node_register()
{
  static bNodeType ntype;
  fn_node_type_base(&ntype, FN_NODE_EASING, "Easing", NODE_CLASS_CONVERTER);
  ntype.declare = node_easing_declare;
  ntype.updatefunc = node_easing_update;
  ntype.initfunc = node_easing_init;
  node_type_storage(&ntype, "NodeEasing", node_free_standard_storage, node_copy_standard_storage);
  ntype.build_multi_function = node_easing_build_multi_function;
  ntype.draw_buttons = node_easing_layout;
  nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_fn_easing_cc
