/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_bit_span.hh"
#include "BLI_span.hh"

namespace blender::bits {

/**
 * Convert the bools to bits. E.g. [true, false, true, true] becomes 1011.
 *
 * \note The caller is responsible for setting the bits to zero before calling this function.
 */
void bools_to_zeroed_bits(Span<bool> bools, MutableBitSpan r_bits);

}  // namespace blender::bits
