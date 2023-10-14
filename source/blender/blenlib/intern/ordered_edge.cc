/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_ordered_edge.hh"

namespace blender {

bool operator==(const OrderedEdge &e1, const OrderedEdge &e2)
{
  BLI_assert(e1.v_low < e1.v_high);
  BLI_assert(e2.v_low < e2.v_high);
  return e1.v_low == e2.v_low && e1.v_high == e2.v_high;
}

bool operator!=(const OrderedEdge &e1, const OrderedEdge &e2)
{
  BLI_assert(e1.v_low < e1.v_high);
  BLI_assert(e2.v_low < e2.v_high);
  return !(e1 == e2);
}

std::ostream &operator<<(std::ostream &stream, const OrderedEdge &e)
{
  return stream << "OrderedEdge(" << e.v_low << ", " << e.v_high << ")";
}

bool operator<(const OrderedEdge &e1, const OrderedEdge &e2)
{
  if (e1.v_low != e2.v_low) {
    return e1.v_low < e2.v_low;
  }
  return e1.v_high < e2.v_high;
}

}  // namespace blender
