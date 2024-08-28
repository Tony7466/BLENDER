/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_bit_span.hh"
#include "BLI_span.hh"

namespace blender::bits {

void bools_to_bits(Span<bool> bools, MutableBitSpan r_bits);

}
