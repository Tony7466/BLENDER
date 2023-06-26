/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "DNA_node_tree_interface_types.h"

#include "BLI_span.hh"

#ifdef __cplusplus
#  include <type_traits>
#endif

#ifdef __cplusplus

inline blender::Span<const bNodeTreeInterfaceItem *> bNodeTreeInterface::items() const
{
  return blender::Span(items_array, items_num);
}

inline blender::MutableSpan<bNodeTreeInterfaceItem *> bNodeTreeInterface::items()
{
  return blender::MutableSpan(items_array, items_num);
}

template<typename T> T &bNodeTreeInterfaceItem::get_as()
{
#  ifndef NDEBUG
  switch (item_type) {
    case NODE_INTERFACE_PANEL: {
      constexpr bool is_valid_type = std::is_same<T, bNodeTreeInterfacePanel>::value;
      BLI_assert(is_valid_type);
      break;
    }
    default:
    case NODE_INTERFACE_SOCKET: {
      constexpr bool is_valid_type = std::is_same<T, bNodeTreeInterfaceSocket>::value;
      BLI_assert(is_valid_type);
      break;
    }
  }
#  endif

  return *reinterpret_cast<T *>(this);
}

template<typename T> const T &bNodeTreeInterfaceItem::get_as() const
{
#  ifndef NDEBUG
  switch (item_type) {
    case NODE_INTERFACE_PANEL: {
      constexpr bool is_valid_type = std::is_same<T, bNodeTreeInterfacePanel>::value;
      BLI_assert(is_valid_type);
      break;
    }
    default:
    case NODE_INTERFACE_SOCKET: {
      constexpr bool is_valid_type = std::is_same<T, bNodeTreeInterfaceSocket>::value;
      BLI_assert(is_valid_type);
      break;
    }
  }
#  endif

  return *reinterpret_cast<const T *>(this);
}

template<typename T> T *bNodeTreeInterfaceItem::get_as_ptr()
{
  switch (item_type) {
    case NODE_INTERFACE_PANEL: {
      constexpr bool is_valid_type = std::is_same<T, bNodeTreeInterfacePanel>::value;
      return is_valid_type ? reinterpret_cast<T *>(this) : nullptr;
    }
    case NODE_INTERFACE_SOCKET: {
      constexpr bool is_valid_type = std::is_same<T, bNodeTreeInterfaceSocket>::value;
      return is_valid_type ? reinterpret_cast<T *>(this) : nullptr;
    }
    default:
      return nullptr;
  }
}

template<typename T> const T *bNodeTreeInterfaceItem::get_as_ptr() const
{
  switch (item_type) {
    case NODE_INTERFACE_PANEL: {
      constexpr bool is_valid_type = std::is_same<T, bNodeTreeInterfacePanel>::value;
      return is_valid_type ? reinterpret_cast<const T *>(this) : nullptr;
    }
    default:
    case NODE_INTERFACE_SOCKET:
      constexpr bool is_valid_type = std::is_same<T, bNodeTreeInterfaceSocket>::value;
      return is_valid_type ? reinterpret_cast<const T *>(this) : nullptr;
  }
}

#endif
