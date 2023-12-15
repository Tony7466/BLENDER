/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 */

#include "BKE_node_socket_value.hh"

namespace blender::bke {

template<typename T> T SocketValueVariant::extract_as()
{
  /* TODO */
  return T();
}
template<typename T> void SocketValueVariant::store_as_impl(T value)
{
  /* TODO */
}

}  // namespace blender::bke
