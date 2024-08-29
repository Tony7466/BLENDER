/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_bit_span.hh"
#include "BLI_span.hh"

namespace blender::bits {

/**
 * Converts the bools to bits and `or`s them into the given bits. For pure conversion, the bits
 * should therefore be zero initialized before they are passed into this function.
 */
void or_bools_into_bits(Span<bool> bools, MutableBitSpan r_bits);

}  // namespace blender::bits
