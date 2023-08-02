/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_idprop.h"
#include "BKE_lib_id.h"
#include "BKE_lib_query.h"
#include "BKE_node.h"
#include "BKE_node_tree_interface.hh"

#include "BLI_math.h"
#include "BLI_stack.hh"
#include "BLI_string.h"
#include "BLI_vector.hh"

#include "BLO_read_write.h"

#include "DNA_collection_types.h"
#include "DNA_material_types.h"
#include "DNA_node_tree_interface_types.h"
#include "DNA_node_types.h"

#include "RNA_access.h"
#include "RNA_prototypes.h"

namespace blender::bke::node_interface {

namespace socket_types {

/* -------------------------------------------------------------------- */
/** \name ID user increment in socket data
 * \{ */

[[maybe_unused]] static void socket_data_id_user_increment(bNodeSocketValueObject &data)
{
  id_us_plus(reinterpret_cast<ID *>(data.value));
}
[[maybe_unused]] static void socket_data_id_user_increment(bNodeSocketValueImage &data)
{
  id_us_plus(reinterpret_cast<ID *>(data.value));
}
[[maybe_unused]] static void socket_data_id_user_increment(bNodeSocketValueCollection &data)
{
  id_us_plus(reinterpret_cast<ID *>(data.value));
}
[[maybe_unused]] static void socket_data_id_user_increment(bNodeSocketValueTexture &data)
{
  id_us_plus(reinterpret_cast<ID *>(data.value));
}
[[maybe_unused]] static void socket_data_id_user_increment(bNodeSocketValueMaterial &data)
{
  id_us_plus(reinterpret_cast<ID *>(data.value));
}
/* Default implementation. */
template<typename T> static void socket_data_id_user_increment(T & /*data*/) {}

/** \} */

/* -------------------------------------------------------------------- */
/** \name ID user decrement in socket data
 * \{ */

[[maybe_unused]] static void socket_data_id_user_decrement(bNodeSocketValueObject &data)
{
  id_us_min(reinterpret_cast<ID *>(data.value));
}
[[maybe_unused]] static void socket_data_id_user_decrement(bNodeSocketValueImage &data)
{
  id_us_min(reinterpret_cast<ID *>(data.value));
}
[[maybe_unused]] static void socket_data_id_user_decrement(bNodeSocketValueCollection &data)
{
  id_us_min(reinterpret_cast<ID *>(data.value));
}
[[maybe_unused]] static void socket_data_id_user_decrement(bNodeSocketValueTexture &data)
{
  id_us_min(reinterpret_cast<ID *>(data.value));
}
[[maybe_unused]] static void socket_data_id_user_decrement(bNodeSocketValueMaterial &data)
{
  id_us_min(reinterpret_cast<ID *>(data.value));
}
/* Default implementation. */
template<typename T> static void socket_data_id_user_decrement(T & /*data*/) {}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Initialize socket data
 * \{ */

[[maybe_unused]] static void socket_data_init_impl(bNodeSocketValueFloat &data)
{
  data.subtype = PROP_NONE;
  data.value = 0.0f;
  data.min = -FLT_MAX;
  data.max = FLT_MAX;
}
[[maybe_unused]] static void socket_data_init_impl(bNodeSocketValueInt &data)
{
  data.subtype = PROP_NONE;
  data.value = 0;
  data.min = INT_MIN;
  data.max = INT_MAX;
}
[[maybe_unused]] static void socket_data_init_impl(bNodeSocketValueBoolean &data)
{
  data.value = false;
}
[[maybe_unused]] static void socket_data_init_impl(bNodeSocketValueRotation & /*data*/) {}
[[maybe_unused]] static void socket_data_init_impl(bNodeSocketValueVector &data)
{
  static float default_value[] = {0.0f, 0.0f, 0.0f};
  data.subtype = PROP_NONE;
  copy_v3_v3(data.value, default_value);
  data.min = -FLT_MAX;
  data.max = FLT_MAX;
}
[[maybe_unused]] static void socket_data_init_impl(bNodeSocketValueRGBA &data)
{
  static float default_value[] = {0.0f, 0.0f, 0.0f, 1.0f};
  copy_v4_v4(data.value, default_value);
}
[[maybe_unused]] static void socket_data_init_impl(bNodeSocketValueString &data)
{
  data.subtype = PROP_NONE;
  data.value[0] = '\0';
}
[[maybe_unused]] static void socket_data_init_impl(bNodeSocketValueObject &data)
{
  data.value = nullptr;
}
[[maybe_unused]] static void socket_data_init_impl(bNodeSocketValueImage &data)
{
  data.value = nullptr;
}
[[maybe_unused]] static void socket_data_init_impl(bNodeSocketValueCollection &data)
{
  data.value = nullptr;
}
[[maybe_unused]] static void socket_data_init_impl(bNodeSocketValueTexture &data)
{
  data.value = nullptr;
}
[[maybe_unused]] static void socket_data_init_impl(bNodeSocketValueMaterial &data)
{
  data.value = nullptr;
}
/* Default implementation */
template<typename T> static void socket_data_init_impl(T & /*data*/) {}

static void *make_socket_data(const char *socket_type)
{
  void *socket_data = nullptr;
  socket_data_to_static_type_tag(socket_type, [&socket_data](auto type_tag) {
    using SocketDataType = typename decltype(type_tag)::type;
    SocketDataType *new_socket_data = MEM_cnew<SocketDataType>(__func__);
    socket_data_init_impl(*new_socket_data);
    socket_data = new_socket_data;
  });
  return socket_data;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Copy allocated socket data
 * \{ */

/* Default implementation */
template<typename T> static void socket_data_copy_impl(T & /*dst*/, const T & /*src*/) {}

static void socket_data_copy(bNodeTreeInterfaceSocket &dst,
                             const bNodeTreeInterfaceSocket &src,
                             int flag)
{
  socket_data_to_static_type_tag(dst.socket_type, [&](auto type_tag) {
    using SocketDataType = typename decltype(type_tag)::type;
    dst.socket_data = MEM_dupallocN(src.socket_data);
    socket_data_copy_impl(get_socket_data_as<SocketDataType>(dst),
                          get_socket_data_as<SocketDataType>(src));
    if ((flag & LIB_ID_CREATE_NO_USER_REFCOUNT) == 0) {
      socket_data_id_user_increment(get_socket_data_as<SocketDataType>(dst));
    }
  });
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Copy allocated socket data
 * \{ */

/* Default implementation */
template<typename T> static void socket_data_free_impl(T & /*data*/, const bool /*do_id_user*/) {}

static void socket_data_free(bNodeTreeInterfaceSocket &socket, const bool do_id_user)
{
  socket_data_to_static_type_tag(socket.socket_type, [&](auto type_tag) {
    using SocketDataType = typename decltype(type_tag)::type;
    if (do_id_user) {
      socket_data_id_user_decrement(get_socket_data_as<SocketDataType>(socket));
    }
    socket_data_free_impl(get_socket_data_as<SocketDataType>(socket), do_id_user);
  });
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Write socket data to blend file
 * \{ */

[[maybe_unused]] static void socket_data_write_impl(BlendWriter *writer,
                                                    bNodeSocketValueFloat &data)
{
  BLO_write_struct(writer, bNodeSocketValueFloat, &data);
}
[[maybe_unused]] static void socket_data_write_impl(BlendWriter *writer, bNodeSocketValueInt &data)
{
  BLO_write_struct(writer, bNodeSocketValueInt, &data);
}
[[maybe_unused]] static void socket_data_write_impl(BlendWriter *writer,
                                                    bNodeSocketValueBoolean &data)
{
  BLO_write_struct(writer, bNodeSocketValueBoolean, &data);
}
[[maybe_unused]] static void socket_data_write_impl(BlendWriter *writer,
                                                    bNodeSocketValueRotation &data)
{
  BLO_write_struct(writer, bNodeSocketValueRotation, &data);
}
[[maybe_unused]] static void socket_data_write_impl(BlendWriter *writer,
                                                    bNodeSocketValueVector &data)
{
  BLO_write_struct(writer, bNodeSocketValueVector, &data);
}
[[maybe_unused]] static void socket_data_write_impl(BlendWriter *writer,
                                                    bNodeSocketValueRGBA &data)
{
  BLO_write_struct(writer, bNodeSocketValueRGBA, &data);
}
[[maybe_unused]] static void socket_data_write_impl(BlendWriter *writer,
                                                    bNodeSocketValueString &data)
{
  BLO_write_struct(writer, bNodeSocketValueString, &data);
}
[[maybe_unused]] static void socket_data_write_impl(BlendWriter *writer,
                                                    bNodeSocketValueObject &data)
{
  BLO_write_struct(writer, bNodeSocketValueObject, &data);
}
[[maybe_unused]] static void socket_data_write_impl(BlendWriter *writer,
                                                    bNodeSocketValueImage &data)
{
  BLO_write_struct(writer, bNodeSocketValueImage, &data);
}
[[maybe_unused]] static void socket_data_write_impl(BlendWriter *writer,
                                                    bNodeSocketValueCollection &data)
{
  BLO_write_struct(writer, bNodeSocketValueCollection, &data);
}
[[maybe_unused]] static void socket_data_write_impl(BlendWriter *writer,
                                                    bNodeSocketValueTexture &data)
{
  BLO_write_struct(writer, bNodeSocketValueTexture, &data);
}
[[maybe_unused]] static void socket_data_write_impl(BlendWriter *writer,
                                                    bNodeSocketValueMaterial &data)
{
  BLO_write_struct(writer, bNodeSocketValueMaterial, &data);
}

/* Note: no default implementation, every used type must write at least the base struct. */

static void socket_data_write(BlendWriter *writer, bNodeTreeInterfaceSocket &socket)
{
  socket_data_to_static_type_tag(socket.socket_type, [&](auto type_tag) {
    using SocketDataType = typename decltype(type_tag)::type;
    socket_data_write_impl(writer, get_socket_data_as<SocketDataType>(socket));
  });
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Read socket data from blend file
 * \{ */

/* Default implementation */
template<typename T> static void socket_data_read_data_impl(BlendDataReader *reader, T **data)
{
  BLO_read_data_address(reader, data);
}

static void socket_data_read_data(BlendDataReader *reader, bNodeTreeInterfaceSocket &socket)
{
  socket_data_to_static_type_tag(socket.socket_type, [&](auto type_tag) {
    using SocketDataType = typename decltype(type_tag)::type;
    socket_data_read_data_impl(reader, reinterpret_cast<SocketDataType **>(&socket.socket_data));
  });
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Read ID pointer data
 * \{ */

[[maybe_unused]] static void socket_data_read_lib_impl(BlendLibReader *reader,
                                                       ID *id,
                                                       bNodeSocketValueObject &data)
{
  BLI_assert(id != nullptr);
  BLO_read_id_address(reader, id, &data.value);
}
[[maybe_unused]] static void socket_data_read_lib_impl(BlendLibReader *reader,
                                                       ID *id,
                                                       bNodeSocketValueImage &data)
{
  BLI_assert(id != nullptr);
  BLO_read_id_address(reader, id, &data.value);
}
[[maybe_unused]] static void socket_data_read_lib_impl(BlendLibReader *reader,
                                                       ID *id,
                                                       bNodeSocketValueCollection &data)
{
  BLI_assert(id != nullptr);
  BLO_read_id_address(reader, id, &data.value);
}
[[maybe_unused]] static void socket_data_read_lib_impl(BlendLibReader *reader,
                                                       ID *id,
                                                       bNodeSocketValueTexture &data)
{
  BLI_assert(id != nullptr);
  BLO_read_id_address(reader, id, &data.value);
}
[[maybe_unused]] static void socket_data_read_lib_impl(BlendLibReader *reader,
                                                       ID *id,
                                                       bNodeSocketValueMaterial &data)
{
  BLI_assert(id != nullptr);
  BLO_read_id_address(reader, id, &data.value);
}
/* Default implementation */
template<typename T>
static void socket_data_read_lib_impl(BlendLibReader * /*reader*/, ID * /*id*/, T & /*data*/)
{
}

static void socket_data_read_lib(BlendLibReader *reader, ID *id, bNodeTreeInterfaceSocket &socket)
{
  socket_data_to_static_type_tag(socket.socket_type, [&](auto type_tag) {
    using SocketDataType = typename decltype(type_tag)::type;
    socket_data_read_lib_impl(reader, id, get_socket_data_as<SocketDataType>(socket));
  });
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Expand socket data
 * \{ */

[[maybe_unused]] static void socket_data_expand_impl(BlendExpander *expander,
                                                     bNodeSocketValueObject &data)
{
  BLO_expand(expander, &data.value);
}
[[maybe_unused]] static void socket_data_expand_impl(BlendExpander *expander,
                                                     bNodeSocketValueImage &data)
{
  BLO_expand(expander, &data.value);
}
[[maybe_unused]] static void socket_data_expand_impl(BlendExpander *expander,
                                                     bNodeSocketValueCollection &data)
{
  BLO_expand(expander, &data.value);
}
[[maybe_unused]] static void socket_data_expand_impl(BlendExpander *expander,
                                                     bNodeSocketValueTexture &data)
{
  BLO_expand(expander, &data.value);
}
[[maybe_unused]] static void socket_data_expand_impl(BlendExpander *expander,
                                                     bNodeSocketValueMaterial &data)
{
  BLO_expand(expander, &data.value);
}
/* Default implementation */
template<typename T>
static void socket_data_expand_impl(BlendExpander * /*expander*/, T & /*data*/)
{
}

static void socket_data_expand(BlendExpander *expander, bNodeTreeInterfaceSocket &socket)
{
  socket_data_to_static_type_tag(socket.socket_type, [&](auto type_tag) {
    using SocketDataType = typename decltype(type_tag)::type;
    socket_data_expand_impl(expander, get_socket_data_as<SocketDataType>(socket));
  });
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Callback per ID pointer
 * \{ */

[[maybe_unused]] static void socket_data_foreach_id_impl(LibraryForeachIDData *cb,
                                                         bNodeSocketValueObject &data)
{
  BKE_LIB_FOREACHID_PROCESS_IDSUPER(cb, data.value, IDWALK_CB_USER);
}
[[maybe_unused]] static void socket_data_foreach_id_impl(LibraryForeachIDData *cb,
                                                         bNodeSocketValueImage &data)
{
  BKE_LIB_FOREACHID_PROCESS_IDSUPER(cb, data.value, IDWALK_CB_USER);
}
[[maybe_unused]] static void socket_data_foreach_id_impl(LibraryForeachIDData *cb,
                                                         bNodeSocketValueCollection &data)
{
  BKE_LIB_FOREACHID_PROCESS_IDSUPER(cb, data.value, IDWALK_CB_USER);
}
[[maybe_unused]] static void socket_data_foreach_id_impl(LibraryForeachIDData *cb,
                                                         bNodeSocketValueTexture &data)
{
  BKE_LIB_FOREACHID_PROCESS_IDSUPER(cb, data.value, IDWALK_CB_USER);
}
[[maybe_unused]] static void socket_data_foreach_id_impl(LibraryForeachIDData *cb,
                                                         bNodeSocketValueMaterial &data)
{
  BKE_LIB_FOREACHID_PROCESS_IDSUPER(cb, data.value, IDWALK_CB_USER);
}
/* Default implementation */
template<typename T>
static void socket_data_foreach_id_impl(LibraryForeachIDData * /*data*/, T & /*data*/)
{
}

static void socket_data_foreach_id(LibraryForeachIDData *data, bNodeTreeInterfaceSocket &socket)
{
  socket_data_to_static_type_tag(socket.socket_type, [&](auto type_tag) {
    using SocketDataType = typename decltype(type_tag)::type;
    socket_data_foreach_id_impl(data, get_socket_data_as<SocketDataType>(socket));
  });
}

/** \} */

}  // namespace socket_types

namespace item_types {

static void item_copy(bNodeTreeInterfaceItem &dst,
                      const bNodeTreeInterfaceItem &src,
                      const int flag)
{
  switch (dst.item_type) {
    case NODE_INTERFACE_SOCKET: {
      bNodeTreeInterfaceSocket &dst_socket = reinterpret_cast<bNodeTreeInterfaceSocket &>(dst);
      const bNodeTreeInterfaceSocket &src_socket =
          reinterpret_cast<const bNodeTreeInterfaceSocket &>(src);
      BLI_assert(src_socket.name != nullptr);
      BLI_assert(src_socket.socket_type != nullptr);

      dst_socket.name = BLI_strdup(src_socket.name);
      dst_socket.description = src_socket.description ? BLI_strdup(src_socket.description) :
                                                        nullptr;
      dst_socket.socket_type = BLI_strdup(src_socket.socket_type);
      dst_socket.default_attribute_name = src_socket.default_attribute_name ?
                                              BLI_strdup(src_socket.default_attribute_name) :
                                              nullptr;
      dst_socket.uid_compat = src_socket.uid_compat ? BLI_strdup(src_socket.uid_compat) : nullptr;
      if (src_socket.prop) {
        dst_socket.prop = IDP_CopyProperty_ex(src_socket.prop, flag);
      }

      if (src_socket.socket_data != nullptr) {
        socket_types::socket_data_copy(dst_socket, src_socket, flag);
      }
      break;
    }
    case NODE_INTERFACE_PANEL: {
      bNodeTreeInterfacePanel &dst_panel = reinterpret_cast<bNodeTreeInterfacePanel &>(dst);
      const bNodeTreeInterfacePanel &src_panel = reinterpret_cast<const bNodeTreeInterfacePanel &>(
          src);
      BLI_assert(src_panel.name != nullptr);

      dst_panel.name = BLI_strdup(src_panel.name);
      dst_panel.copy_items(src_panel.items(), flag);
      break;
    }
  }
}

static void item_free(bNodeTreeInterfaceItem &item, const bool do_id_user)
{
  switch (item.item_type) {
    case NODE_INTERFACE_SOCKET: {
      bNodeTreeInterfaceSocket &socket = reinterpret_cast<bNodeTreeInterfaceSocket &>(item);

      if (socket.socket_data != nullptr) {
        socket_types::socket_data_free(socket, do_id_user);
        MEM_SAFE_FREE(socket.socket_data);
      }

      MEM_SAFE_FREE(socket.name);
      MEM_SAFE_FREE(socket.description);
      MEM_SAFE_FREE(socket.socket_type);
      MEM_SAFE_FREE(socket.default_attribute_name);
      MEM_SAFE_FREE(socket.uid_compat);
      if (socket.prop) {
        IDP_FreePropertyContent_ex(socket.prop, do_id_user);
        MEM_freeN(socket.prop);
      }
      break;
    }
    case NODE_INTERFACE_PANEL: {
      bNodeTreeInterfacePanel &panel = reinterpret_cast<bNodeTreeInterfacePanel &>(item);

      panel.clear_items();
      MEM_SAFE_FREE(panel.name);
      break;
    }
  }

  MEM_freeN(&item);
}

static void item_write_struct(BlendWriter *writer, bNodeTreeInterfaceItem &item);

static void item_write_data(BlendWriter *writer, bNodeTreeInterfaceItem &item)
{
  switch (item.item_type) {
    case NODE_INTERFACE_SOCKET: {
      bNodeTreeInterfaceSocket &socket = reinterpret_cast<bNodeTreeInterfaceSocket &>(item);
      BLO_write_string(writer, socket.name);
      BLO_write_string(writer, socket.description);
      BLO_write_string(writer, socket.socket_type);
      BLO_write_string(writer, socket.default_attribute_name);
      if (socket.prop) {
        IDP_BlendWrite(writer, socket.prop);
      }

      socket_types::socket_data_write(writer, socket);
      break;
    }
    case NODE_INTERFACE_PANEL: {
      bNodeTreeInterfacePanel &panel = reinterpret_cast<bNodeTreeInterfacePanel &>(item);
      BLO_write_string(writer, panel.name);
      BLO_write_pointer_array(writer, panel.items_num, panel.items_array);
      for (bNodeTreeInterfaceItem *child_item : panel.items()) {
        item_write_struct(writer, *child_item);
      }
      break;
    }
  }
}

static void item_write_struct(BlendWriter *writer, bNodeTreeInterfaceItem &item)
{
  switch (item.item_type) {
    case NODE_INTERFACE_SOCKET: {
      BLO_write_struct(writer, bNodeTreeInterfaceSocket, &item);
      break;
    }
    case NODE_INTERFACE_PANEL: {
      BLO_write_struct(writer, bNodeTreeInterfacePanel, &item);
      break;
    }
  }

  item_write_data(writer, item);
}

static void item_read_data(BlendDataReader *reader, bNodeTreeInterfaceItem &item)
{
  switch (item.item_type) {
    case NODE_INTERFACE_SOCKET: {
      bNodeTreeInterfaceSocket &socket = reinterpret_cast<bNodeTreeInterfaceSocket &>(item);
      BLO_read_data_address(reader, &socket.name);
      BLO_read_data_address(reader, &socket.description);
      BLO_read_data_address(reader, &socket.socket_type);
      BLO_read_data_address(reader, &socket.default_attribute_name);
      /* Only used for backwards compatibility. */
      socket.uid_compat = nullptr;
      BLO_read_data_address(reader, &socket.prop);
      IDP_BlendDataRead(reader, &socket.prop);

      socket_types::socket_data_read_data(reader, socket);
      break;
    }
    case NODE_INTERFACE_PANEL: {
      bNodeTreeInterfacePanel &panel = reinterpret_cast<bNodeTreeInterfacePanel &>(item);
      BLO_read_data_address(reader, &panel.name);
      BLO_read_pointer_array(reader, reinterpret_cast<void **>(&panel.items_array));
      for (const int i : blender::IndexRange(panel.items_num)) {
        BLO_read_data_address(reader, &panel.items_array[i]);
        item_read_data(reader, *panel.items_array[i]);
      }
      break;
    }
  }
}

static void item_read_lib(BlendLibReader *reader, ID *id, bNodeTreeInterfaceItem &item)
{
  switch (item.item_type) {
    case NODE_INTERFACE_SOCKET: {
      bNodeTreeInterfaceSocket &socket = reinterpret_cast<bNodeTreeInterfaceSocket &>(item);
      IDP_BlendReadLib(reader, id, socket.prop);
      socket_types::socket_data_read_lib(reader, id, socket);
      break;
    }
    case NODE_INTERFACE_PANEL: {
      bNodeTreeInterfacePanel &panel = reinterpret_cast<bNodeTreeInterfacePanel &>(item);
      for (bNodeTreeInterfaceItem *item : panel.items()) {
        item_read_lib(reader, id, *item);
      }
      break;
    }
  }
}

static void item_read_expand(BlendExpander *expander, bNodeTreeInterfaceItem &item)
{
  switch (item.item_type) {
    case NODE_INTERFACE_SOCKET: {
      bNodeTreeInterfaceSocket &socket = reinterpret_cast<bNodeTreeInterfaceSocket &>(item);
      IDP_BlendReadExpand(expander, socket.prop);
      socket_types::socket_data_expand(expander, socket);
      break;
    }
    case NODE_INTERFACE_PANEL: {
      bNodeTreeInterfacePanel &panel = reinterpret_cast<bNodeTreeInterfacePanel &>(item);
      for (bNodeTreeInterfaceItem *item : panel.items()) {
        item_read_expand(expander, *item);
      }
      break;
    }
  }
}

static void item_foreach_id(LibraryForeachIDData *data, bNodeTreeInterfaceItem &item)
{
  switch (item.item_type) {
    case NODE_INTERFACE_SOCKET: {
      bNodeTreeInterfaceSocket &socket = reinterpret_cast<bNodeTreeInterfaceSocket &>(item);

      BKE_LIB_FOREACHID_PROCESS_FUNCTION_CALL(
          data,
          IDP_foreach_property(socket.prop,
                               IDP_TYPE_FILTER_ID,
                               BKE_lib_query_idpropertiesForeachIDLink_callback,
                               data));

      socket_types::socket_data_foreach_id(data, socket);
      break;
    }
    case NODE_INTERFACE_PANEL: {
      bNodeTreeInterfacePanel &panel = reinterpret_cast<bNodeTreeInterfacePanel &>(item);
      for (bNodeTreeInterfaceItem *item : panel.items()) {
        item_foreach_id(data, *item);
      }
      break;
    }
  }
}

}  // namespace item_types

}  // namespace blender::bke::node_interface

using namespace blender::bke::node_interface;

std::string bNodeTreeInterfaceSocket::socket_identifier() const
{
  return uid_compat ? uid_compat : "Socket" + std::to_string(uid);
}

bNodeSocketType *bNodeTreeInterfaceSocket::socket_typeinfo() const
{
  return nodeSocketTypeFind(socket_type);
}

blender::ColorGeometry4f bNodeTreeInterfaceSocket::socket_color() const
{
  bNodeSocketType *typeinfo = socket_typeinfo();
  if (!typeinfo || !typeinfo->draw_color) {
    return blender::ColorGeometry4f(1.0f, 0.0f, 1.0f, 1.0f);
  }

  /* XXX Warning! Color callbacks for sockets are overly general,
   * using instance pointers for potential context-based colors.
   * This is not used by the standard socket types, but might be used
   * by python nodes.
   *
   * There should be a simplified "base color" callback that does not
   * depend on context, for drawing socket type colors without any
   * actual socket instance. We just set pointers to null here and
   * hope for the best until the situation can be improved ...
   */
  //    bNode dummy_node{0};
  bNodeSocket dummy_socket{0};
  dummy_socket.typeinfo = typeinfo;
  PointerRNA socket_ptr, node_ptr;
  //    RNA_pointer_create(nullptr, &RNA_Node, &dummy_node, &node_ptr);
  node_ptr = PointerRNA_NULL;
  RNA_pointer_create(nullptr, &RNA_NodeSocket, &dummy_socket, &socket_ptr);
  bContext *C = nullptr;
  float color[4];
  typeinfo->draw_color(C, &socket_ptr, &node_ptr, color);
  return blender::ColorGeometry4f(color);
}

bool bNodeTreeInterfaceSocket::set_socket_type(const char *new_socket_type)
{
  if (socket_data != nullptr) {
    socket_types::socket_data_free(*this, true);
    MEM_SAFE_FREE(socket_data);
  }
  MEM_SAFE_FREE(socket_type);

  socket_type = BLI_strdup(new_socket_type);
  socket_data = socket_types::make_socket_data(new_socket_type);

  return true;
}

blender::IndexRange bNodeTreeInterfacePanel::items_range() const
{
  return blender::IndexRange(items_num);
}

blender::Span<const bNodeTreeInterfaceItem *> bNodeTreeInterfacePanel::items() const
{
  return blender::Span(items_array, items_num);
}

blender::MutableSpan<bNodeTreeInterfaceItem *> bNodeTreeInterfacePanel::items()
{
  return blender::MutableSpan(items_array, items_num);
}

int bNodeTreeInterfacePanel::item_index(const bNodeTreeInterfaceItem &item) const
{
  return items().first_index_try(&item);
}

bool bNodeTreeInterfacePanel::contains_item(const bNodeTreeInterfaceItem &item) const
{
  return items().contains(&item);
}

bool bNodeTreeInterfacePanel::find_item(const bNodeTreeInterfaceItem &item) const
{
  bool is_child = false;
  /* Have to capture item address here instead of just a reference,
   * otherwise pointer comparison will not work. */
  foreach_item(
      [&item, &is_child](const bNodeTreeInterfaceItem &titem) {
        if (&titem == &item) {
          is_child = true;
          return false;
        }
        return true;
      },
      true);
  return is_child;
}

int bNodeTreeInterfacePanel::find_item_index(const bNodeTreeInterfaceItem &item) const
{
  int index = 0;
  bool found = false;
  /* Have to capture item address here instead of just a reference,
   * otherwise pointer comparison will not work. */
  foreach_item([&item, &index, &found](const bNodeTreeInterfaceItem &titem) {
    if (&titem == &item) {
      found = true;
      return false;
    }
    ++index;
    return true;
  });
  return found ? index : -1;
}

const bNodeTreeInterfaceItem *bNodeTreeInterfacePanel::get_item_at_index(int index) const
{
  int i = 0;
  const bNodeTreeInterfaceItem *result = nullptr;
  foreach_item([&i, index, &result](const bNodeTreeInterfaceItem &item) {
    if (i == index) {
      result = &item;
      return false;
    }
    ++i;
    return true;
  });
  return result;
}

bool bNodeTreeInterfacePanel::find_item_parent(const bNodeTreeInterfaceItem &item,
                                               bNodeTreeInterfacePanel *&r_parent)
{
  std::queue<bNodeTreeInterfacePanel *> queue;

  if (contains_item(item)) {
    r_parent = this;
    return true;
  }
  queue.push(this);

  while (!queue.empty()) {
    bNodeTreeInterfacePanel *parent = queue.front();
    queue.pop();

    for (bNodeTreeInterfaceItem *titem : parent->items()) {
      if (titem->item_type != NODE_INTERFACE_PANEL) {
        continue;
      }

      bNodeTreeInterfacePanel *tpanel = get_item_as<bNodeTreeInterfacePanel>(titem);
      if (tpanel->contains_item(item)) {
        r_parent = tpanel;
        return true;
      }
      queue.push(tpanel);
    }
  }

  r_parent = nullptr;
  return false;
}

void bNodeTreeInterfacePanel::add_item(bNodeTreeInterfaceItem &item)
{
  blender::MutableSpan<bNodeTreeInterfaceItem *> old_items = items();
  items_num++;
  items_array = MEM_cnew_array<bNodeTreeInterfaceItem *>(items_num, __func__);
  items().drop_back(1).copy_from(old_items);
  items().last() = &item;

  if (old_items.data()) {
    MEM_freeN(old_items.data());
  }
}

void bNodeTreeInterfacePanel::insert_item(bNodeTreeInterfaceItem &item, int index)
{
  index = std::min(std::max(index, 0), items_num);

  blender::MutableSpan<bNodeTreeInterfaceItem *> old_items = items();
  items_num++;
  items_array = MEM_cnew_array<bNodeTreeInterfaceItem *>(items_num, __func__);
  items().take_front(index).copy_from(old_items.take_front(index));
  items().drop_front(index + 1).copy_from(old_items.drop_front(index));
  items()[index] = &item;

  if (old_items.data()) {
    MEM_freeN(old_items.data());
  }
}

bool bNodeTreeInterfacePanel::remove_item(bNodeTreeInterfaceItem &item, bool free)
{
  const int index = item_index(item);
  if (!items().index_range().contains(index)) {
    return false;
  }

  blender::MutableSpan<bNodeTreeInterfaceItem *> old_items = items();
  items_num--;
  items_array = MEM_cnew_array<bNodeTreeInterfaceItem *>(items_num, __func__);
  items().take_front(index).copy_from(old_items.take_front(index));
  items().drop_front(index).copy_from(old_items.drop_front(index + 1));

  /* Guaranteed not empty, contains at least the removed item */
  MEM_freeN(old_items.data());

  if (free) {
    item_types::item_free(item, true);
  }

  return true;
}

void bNodeTreeInterfacePanel::clear_items()
{
  for (bNodeTreeInterfaceItem *item : items()) {
    item_types::item_free(*item, true);
  }
  MEM_SAFE_FREE(items_array);
  items_array = nullptr;
  items_num = 0;
}

bool bNodeTreeInterfacePanel::move_item(bNodeTreeInterfaceItem &item, const int new_index)
{
  const int old_index = item_index(item);
  if (!items().index_range().contains(old_index) || !items().index_range().contains(new_index)) {
    return false;
  }

  if (old_index == new_index) {
    /* Nothing changes. */
    return true;
  }
  else if (old_index < new_index) {
    const blender::Span<bNodeTreeInterfaceItem *> moved_items = items().slice(
        old_index + 1, new_index - old_index);
    bNodeTreeInterfaceItem *tmp = items()[old_index];
    std::copy(moved_items.begin(), moved_items.end(), items().drop_front(old_index).data());
    items()[new_index] = tmp;
  }
  else /* old_index > new_index */ {
    const blender::Span<bNodeTreeInterfaceItem *> moved_items = items().slice(
        new_index, old_index - new_index);
    bNodeTreeInterfaceItem *tmp = items()[old_index];
    std::copy_backward(
        moved_items.begin(), moved_items.end(), items().drop_front(old_index + 1).data());
    items()[new_index] = tmp;
  }

  return true;
}

void bNodeTreeInterfacePanel::foreach_item(
    blender::FunctionRef<bool(bNodeTreeInterfaceItem &item)> fn, bool include_self)
{
  using ItemSpan = blender::Span<bNodeTreeInterfaceItem *>;
  blender::Stack<ItemSpan> stack;

  if (include_self && fn(this->item) == false) {
    return;
  }
  stack.push(items());

  while (!stack.is_empty()) {
    const ItemSpan current_items = stack.pop();

    for (const int index : current_items.index_range()) {
      bNodeTreeInterfaceItem *item = current_items[index];
      if (fn(*item) == false) {
        return;
      }

      if (item->item_type == NODE_INTERFACE_PANEL) {
        bNodeTreeInterfacePanel *panel = reinterpret_cast<bNodeTreeInterfacePanel *>(item);
        /* Reinsert remaining items. */
        if (index < current_items.size() - 1) {
          const ItemSpan remaining_items = current_items.drop_front(index + 1);
          stack.push(remaining_items);
        }
        /* Handle child items first before continuing with current span. */
        stack.push(panel->items());
        break;
      }
    }
  }
}

void bNodeTreeInterfacePanel::foreach_item(
    blender::FunctionRef<bool(const bNodeTreeInterfaceItem &item)> fn, bool include_self) const
{
  using ItemSpan = blender::Span<const bNodeTreeInterfaceItem *>;
  blender::Stack<ItemSpan> stack;

  if (include_self && fn(this->item) == false) {
    return;
  }
  stack.push(items());

  while (!stack.is_empty()) {
    const ItemSpan current_items = stack.pop();

    for (const int index : current_items.index_range()) {
      const bNodeTreeInterfaceItem *item = current_items[index];
      if (fn(*item) == false) {
        return;
      }

      if (item->item_type == NODE_INTERFACE_PANEL) {
        const bNodeTreeInterfacePanel *panel = reinterpret_cast<const bNodeTreeInterfacePanel *>(
            item);
        /* Reinsert remaining items. */
        if (index < current_items.size() - 1) {
          const ItemSpan remaining_items = current_items.drop_front(index + 1);
          stack.push(remaining_items);
        }
        /* Handle child items first before continuing with current span. */
        stack.push(panel->items());
        break;
      }
    }
  }
}

static bNodeTreeInterfaceSocket *make_socket(const int uid,
                                             blender::StringRef name,
                                             blender::StringRef description,
                                             blender::StringRef socket_type,
                                             const eNodeTreeInterfaceSocketFlag flag)
{
  BLI_assert(name.data() != nullptr);
  BLI_assert(socket_type.data() != nullptr);

  bNodeTreeInterfaceSocket *new_socket = MEM_cnew<bNodeTreeInterfaceSocket>(__func__);
  BLI_assert(new_socket);

  /* Init common socket properties. */
  new_socket->uid = uid;
  new_socket->item.item_type = NODE_INTERFACE_SOCKET;
  new_socket->name = BLI_strdup(name.data());
  new_socket->description = description.data() ? BLI_strdup(description.data()) : nullptr;
  new_socket->socket_type = BLI_strdup(socket_type.data());
  new_socket->flag = flag;

  new_socket->socket_data = socket_types::make_socket_data(socket_type.data());

  return new_socket;
}

static bNodeTreeInterfacePanel *make_panel(const int uid, blender::StringRef name)
{
  BLI_assert(name.data() != nullptr);

  bNodeTreeInterfacePanel *new_panel = MEM_cnew<bNodeTreeInterfacePanel>(__func__);
  new_panel->item.item_type = NODE_INTERFACE_PANEL;
  new_panel->name = BLI_strdup(name.data());
  new_panel->uid = uid;
  return new_panel;
}

void bNodeTreeInterfacePanel::copy_items(
    const blender::Span<const bNodeTreeInterfaceItem *> items_src, int flag)
{
  items_num = items_src.size();
  items_array = MEM_cnew_array<bNodeTreeInterfaceItem *>(items_num, __func__);

  /* Copy buffers. */
  for (const int i : items_src.index_range()) {
    const bNodeTreeInterfaceItem *item_src = items_src[i];
    items_array[i] = static_cast<bNodeTreeInterfaceItem *>(MEM_dupallocN(item_src));
    item_types::item_copy(*items_array[i], *item_src, flag);
  }
}

void bNodeTreeInterface::copy_data(const bNodeTreeInterface &src, int flag)
{
  root_panel.copy_items(src.root_panel.items(), flag);
  this->active_index = src.active_index;
}

void bNodeTreeInterface::free_data()
{
  root_panel.clear_items();
}

bNodeTreeInterfaceSocket *bNodeTreeInterface::add_socket(blender::StringRef name,
                                                         blender::StringRef description,
                                                         blender::StringRef socket_type,
                                                         const eNodeTreeInterfaceSocketFlag flag,
                                                         bNodeTreeInterfacePanel *parent)
{
  if (parent == nullptr) {
    parent = &root_panel;
  }
  BLI_assert(find_item(parent->item));

  bNodeTreeInterfaceSocket *new_socket = make_socket(
      next_uid++, name, description, socket_type, flag);
  parent->add_item(new_socket->item);
  return new_socket;
}

bNodeTreeInterfaceSocket *bNodeTreeInterface::insert_socket(
    blender::StringRef name,
    blender::StringRef description,
    blender::StringRef socket_type,
    const eNodeTreeInterfaceSocketFlag flag,
    bNodeTreeInterfacePanel *parent,
    const int index)
{
  if (parent == nullptr) {
    parent = &root_panel;
  }
  BLI_assert(find_item(parent->item));

  bNodeTreeInterfaceSocket *new_socket = make_socket(
      next_uid++, name, description, socket_type, flag);
  parent->insert_item(new_socket->item, index);
  return new_socket;
}

bNodeTreeInterfacePanel *bNodeTreeInterface::add_panel(blender::StringRef name,
                                                       bNodeTreeInterfacePanel *parent)
{
  if (parent == nullptr) {
    parent = &root_panel;
  }
  BLI_assert(find_item(parent->item));

  bNodeTreeInterfacePanel *new_panel = make_panel(next_uid++, name);
  parent->add_item(new_panel->item);
  return new_panel;
}

bNodeTreeInterfacePanel *bNodeTreeInterface::insert_panel(blender::StringRef name,
                                                          bNodeTreeInterfacePanel *parent,
                                                          const int index)
{
  if (parent == nullptr) {
    parent = &root_panel;
  }
  BLI_assert(find_item(parent->item));

  bNodeTreeInterfacePanel *new_panel = make_panel(next_uid++, name);
  parent->insert_item(new_panel->item, index);
  return new_panel;
}

bNodeTreeInterfaceItem *bNodeTreeInterface::add_item_copy(const bNodeTreeInterfaceItem &item,
                                                          bNodeTreeInterfacePanel *parent)
{
  if (parent == nullptr) {
    parent = &root_panel;
  }
  BLI_assert(find_item(item));
  BLI_assert(find_item(parent->item));

  if (parent == nullptr) {
    parent = &root_panel;
  }

  bNodeTreeInterfaceItem *citem = static_cast<bNodeTreeInterfaceItem *>(MEM_dupallocN(&item));
  item_types::item_copy(*citem, item, 0);
  parent->add_item(*citem);

  return citem;
}

bNodeTreeInterfaceItem *bNodeTreeInterface::insert_item_copy(const bNodeTreeInterfaceItem &item,
                                                             bNodeTreeInterfacePanel *parent,
                                                             int index)
{
  if (parent == nullptr) {
    parent = &root_panel;
  }
  BLI_assert(find_item(item));
  BLI_assert(find_item(parent->item));

  bNodeTreeInterfaceItem *citem = static_cast<bNodeTreeInterfaceItem *>(MEM_dupallocN(&item));
  item_types::item_copy(*citem, item, 0);
  parent->insert_item(*citem, index);

  return citem;
}

bool bNodeTreeInterface::remove_item(bNodeTreeInterfaceItem &item)
{
  bNodeTreeInterfacePanel *parent;
  if (!find_item_parent(item, parent)) {
    return false;
  }
  if (parent->remove_item(item, true)) {
    return true;
  }
  return false;
}

void bNodeTreeInterface::clear_items()
{
  root_panel.clear_items();
}

bool bNodeTreeInterface::move_item(bNodeTreeInterfaceItem &item, const int new_index)
{
  bNodeTreeInterfacePanel *parent;
  if (!find_item_parent(item, parent)) {
    return false;
  }
  return parent->move_item(item, new_index);
}

bool bNodeTreeInterface::move_item_to_parent(bNodeTreeInterfaceItem &item,
                                             bNodeTreeInterfacePanel *new_parent,
                                             int new_index)
{
  bNodeTreeInterfacePanel *parent;
  if (!find_item_parent(item, parent)) {
    return false;
  }
  if (parent->remove_item(item, false)) {
    new_parent->insert_item(item, new_index);
    return true;
  }
  return false;
}

void bNodeTreeInterface::foreach_id(LibraryForeachIDData *cb)
{
  item_types::item_foreach_id(cb, root_panel.item);
}

namespace blender::bke {

void bNodeTreeInterfaceCache::rebuild(bNodeTreeInterface &interface)
{
  /* Rebuild draw-order list of interface items for linear access. */
  items.clear();
  inputs.clear();
  outputs.clear();

  interface.foreach_item([&](bNodeTreeInterfaceItem &item) {
    items.append(&item);
    if (bNodeTreeInterfaceSocket *socket = get_item_as<bNodeTreeInterfaceSocket>(&item)) {
      if (socket->flag & NODE_INTERFACE_SOCKET_INPUT) {
        inputs.append(socket);
      }
      if (socket->flag & NODE_INTERFACE_SOCKET_OUTPUT) {
        outputs.append(socket);
      }
    }
    return true;
  });
}

}  // namespace blender::bke

void BKE_nodetree_interface_init(bNodeTreeInterface * /*interface*/) {}

void BKE_nodetree_interface_copy(bNodeTreeInterface *interface_dst,
                                 const bNodeTreeInterface *interface_src,
                                 int flag)
{
  interface_dst->copy_data(*interface_src, flag);
}

void BKE_nodetree_interface_free(bNodeTreeInterface *interface)
{
  interface->free_data();
}

void BKE_nodetree_interface_write(BlendWriter *writer, bNodeTreeInterface *interface)
{
  BLO_write_struct(writer, bNodeTreeInterface, interface);
  /* Don't write the root panel struct itself, it's nested in the interface struct. */
  item_types::item_write_data(writer, interface->root_panel.item);
}

void BKE_nodetree_interface_read_data(BlendDataReader *reader, bNodeTreeInterface *interface)
{
  item_types::item_read_data(reader, interface->root_panel.item);
}

void BKE_nodetree_interface_read_lib(BlendLibReader *reader, ID *id, bNodeTreeInterface *interface)
{
  item_types::item_read_lib(reader, id, interface->root_panel.item);
}

void BKE_nodetree_interface_read_expand(BlendExpander *expander, bNodeTreeInterface *interface)
{
  item_types::item_read_expand(expander, interface->root_panel.item);
}
