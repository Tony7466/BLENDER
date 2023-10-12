/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <functional>

#include "BLI_function_ref.hh"
#include "BLI_string_ref.hh"
#include "BLI_vector.hh"

#include "DNA_node_types.h" /* Necessary for eNodeSocketInOut. */
#include "NOD_node_declaration.hh"

#include "NOD_node_declaration.hh"

struct bContext;
struct SpaceNode;

namespace blender::nodes {

/**
 * Parameters for the operation of adding a node after the link drag search menu closes.
 */
class LinkSearchOpParams {
 private:
  /**
   * Keep track of the nodes added by the callback, so they can be selected or moved afterwards.
   */
  Vector<bNode *> &added_nodes_;

 public:
  const bContext &C;
  bNodeTree &node_tree;
  /**
   * The node that contains the #socket.
   */
  bNode &node;
  /**
   * The existing socket to connect any added nodes to. Might be an input or output socket.
   */
  bNodeSocket &socket;

  LinkSearchOpParams(const bContext &C,
                     bNodeTree &node_tree,
                     bNode &node,
                     bNodeSocket &socket,
                     Vector<bNode *> &added_nodes)
      : added_nodes_(added_nodes), C(C), node_tree(node_tree), node(node), socket(socket)
  {
  }

  bNode &add_node(StringRef idname);
  bNode &add_node(const bNodeType &type);
  /**
   * Find a socket with the given name (correctly checks for inputs and outputs)
   * and connect it to the socket the link drag started from (#socket).
   */
  void connect_available_socket(bNode &new_node, StringRef socket_name);
  /**
   * Like #connect_available_socket, but also calls the node's update function.
   */
  void update_and_connect_available_socket(bNode &new_node, StringRef socket_name);
};

struct SocketLinkOperation {
  using LinkSocketFn = std::function<void(LinkSearchOpParams &link_params)>;

  std::string name;
  LinkSocketFn fn;
  int weight = 0;
};

class GatherLinkSearchOpParams {
  /** The current node type. */
  const bNodeType &node_type_;

  const SpaceNode &snode_;
  const bNodeTree &node_tree_;

  const bNodeSocket &other_socket_;

  /* The operations currently being built. Owned by the caller. */
  Vector<SocketLinkOperation> &items_;

 public:
  GatherLinkSearchOpParams(const bNodeType &node_type,
                           const SpaceNode &snode,
                           const bNodeTree &node_tree,
                           const bNodeSocket &other_socket,
                           Vector<SocketLinkOperation> &items)
      : node_type_(node_type),
        snode_(snode),
        node_tree_(node_tree),
        other_socket_(other_socket),
        items_(items)
  {
  }

  /**
   * The node on the other side of the dragged link.
   */
  const bNodeSocket &other_socket() const;

  /**
   * The currently active node editor.
   */
  const SpaceNode &space_node() const;

  /**
   * The node tree the user is editing when the search menu is created.
   */
  const bNodeTree &node_tree() const;

  /**
   * The type of the node in the current callback.
   */
  const bNodeType &node_type() const;

  /**
   * Whether to list the input or output sockets of the node.
   */
  eNodeSocketInOut in_out() const;

  /**
   * \param weight: Used to customize the order when multiple search items match.
   *
   * \warning When creating lambdas for the #fn argument, be careful not to capture this class
   * itself, since it is temporary. That is why we tend to use the same variable name for this
   * class (`params`) that we do for the argument to `LinkSocketFn`.
   */
  void add_item(std::string socket_name, SocketLinkOperation::LinkSocketFn fn, int weight = 0);
};

/**
 * This callback can be used for a node type when a few things are true about its inputs.
 * To avoid creating more boilerplate, it is the default callback for node types.
 * - Either all declared sockets are visible in the default state of the node, *OR* the node's
 *   type's declaration has been extended with #make_available functions for those sockets.
 *
 * If a node type does not meet these criteria, the function will do nothing in a release build.
 * In a debug build, an assert will most likely be hit.
 *
 * \note For nodes with the deprecated #bNodeSocketTemplate instead of a declaration,
 * these criteria do not apply and the function just tries its best without asserting.
 */
void search_link_ops_for_basic_node(GatherLinkSearchOpParams &params);

void search_link_ops_for_declarations(GatherLinkSearchOpParams &params,
                                      Span<SocketDeclaration *> declarations);

void search_link_ops_for_declaration(GatherLinkSearchOpParams &params,
                                     FunctionRef<void(NodeDeclarationBuilder &b)> builder);

class DynamicGatherBuilder {
 private:
  GatherLinkSearchOpParams &params_;
  const Span<eNodeSocketDatatype> supported_types_;
  const eNodeSocketDatatype default_dynamic_type_ = SOCK_FLOAT;

 public:
  DynamicGatherBuilder(GatherLinkSearchOpParams &params) : params_(params) {}
  DynamicGatherBuilder(GatherLinkSearchOpParams &params,
                       const Span<eNodeSocketDatatype> supported_types)
      : params_(params), supported_types_(supported_types)
  {
  }
  DynamicGatherBuilder(GatherLinkSearchOpParams &params,
                       const Span<eNodeSocketDatatype> supported_types,
                       const eNodeSocketDatatype default_dynamic_type)
      : params_(params),
        supported_types_(supported_types),
        default_dynamic_type_(default_dynamic_type)
  {
  }

  struct Item {
    const bNodeType &node_type;
    std::string socket_name;
    eNodeSocketDatatype socket_type;

    void operator()(LinkSearchOpParams &params) const
    {
      printf("%p: %d;\n", &node_type, int(socket_type));
      bNode &node = params.add_node(node_type);
      const eCustomDataType type = *node_data_type_to_custom_data_type(socket_type);
      node.custom1 = type;
      params.update_and_connect_available_socket(node, socket_name);
    }
  };

  void add_input(const eNodeSocketDatatype socket_type, std::string name)
  {
    if (params_.in_out() != SOCK_IN) {
      return;
    }
    const bNodeType &node_type = params_.node_type();
    const eNodeSocketDatatype other_type = eNodeSocketDatatype(params_.other_socket().type);
    if (this->connecteble_types(other_type, socket_type) &&
        supported_types_.contains(default_dynamic_type_))
    {
      params_.add_item(name.data(), Item{node_type, name, default_dynamic_type_});
    }
  }

  void add_dynamic_input(const std::string name)
  {
    if (params_.in_out() != SOCK_IN) {
      return;
    }
    const bNodeType *node_type = &params_.node_type();
    const eNodeSocketDatatype other_type = eNodeSocketDatatype(params_.other_socket().type);
    if (supported_types_.contains(other_type)) {
      params_.add_item(IFACE_(name.data()), [=](LinkSearchOpParams &params) {
        bNode &node = params.add_node(*node_type);
        const eCustomDataType type = *node_data_type_to_custom_data_type(other_type);
        node.custom1 = type;
        params.update_and_connect_available_socket(node, name);
      });
    }
  }

  void add_dynamic_output(const std::string name)
  {
    if (params_.in_out() != SOCK_OUT) {
      return;
    }
    const bNodeType *node_type = &params_.node_type();
    const eNodeSocketDatatype other_type = eNodeSocketDatatype(params_.other_socket().type);
    if (supported_types_.contains(other_type)) {
      params_.add_item(IFACE_(name.data()), [=](LinkSearchOpParams &params) {
        bNode &node = params.add_node(*node_type);
        const eCustomDataType type = *node_data_type_to_custom_data_type(other_type);
        node.custom1 = type;
        params.update_and_connect_available_socket(node, name);
      });
    }
  }

 private:
  bool connecteble_types(eNodeSocketDatatype a, eNodeSocketDatatype b) const
  {
    if (ELEM(a,
             SOCK_STRING,
             SOCK_OBJECT,
             SOCK_IMAGE,
             SOCK_GEOMETRY,
             SOCK_COLLECTION,
             SOCK_TEXTURE,
             SOCK_MATERIAL,
             SOCK_ROTATION) ||
        ELEM(b,
             SOCK_STRING,
             SOCK_OBJECT,
             SOCK_IMAGE,
             SOCK_GEOMETRY,
             SOCK_COLLECTION,
             SOCK_TEXTURE,
             SOCK_MATERIAL,
             SOCK_ROTATION))
    {
      return a == b;
    }
    return true;
  }

  static std::optional<eCustomDataType> node_data_type_to_custom_data_type(
      const eNodeSocketDatatype type)
  {
    switch (type) {
      case SOCK_FLOAT:
        return CD_PROP_FLOAT;
      case SOCK_VECTOR:
        return CD_PROP_FLOAT3;
      case SOCK_RGBA:
        return CD_PROP_COLOR;
      case SOCK_BOOLEAN:
        return CD_PROP_BOOL;
      case SOCK_ROTATION:
        return CD_PROP_QUATERNION;
      case SOCK_INT:
        return CD_PROP_INT32;
      case SOCK_STRING:
        return CD_PROP_STRING;
      default:
        return {};
    }
  }
};

}  // namespace blender::nodes
