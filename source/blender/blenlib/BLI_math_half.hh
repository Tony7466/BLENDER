/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bli
 */

#include <cstdint>

namespace blender::math {

/**
 * Converts float (FP32) number to half-precision (FP16).
 * Behavior matches x64 hardware F16C, including handling
 * of denormals, infinities, NaNs etc.
 */
uint16_t float_to_half(float v);

/**
 * Converts half-precision (FP16) number to float (FP32).
 * Behavior matches x64 hardware F16C in default
 * round-to-nearest-even rounding mode, including handling
 * of denormals, infinities, NaNs etc. For NaNs, the exact
 * bit pattern might be differnet but it will still be a NaN.
 */
float half_to_float(uint16_t v);

}  // namespace blender::math
