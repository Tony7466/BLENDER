/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bke
 * \brief Low-level operations for grease pencil.
 */

#include "BLI_function_ref.hh"
#include "BLI_map.hh"
#include "BLI_stack.hh"
#include "BLI_string.h"

#include "DNA_gpencil_legacy_types.h"
#include "DNA_grease_pencil_types.h"

namespace blender::bke {

namespace gpencil {

class LayerGroup;
class Layer;

class TreeNode : public ::GreasePencilLayerTreeNode {
  using ItemIterFn = FunctionRef<void(TreeNode &)>;

 protected:
  Vector<TreeNode, 0> children_;

 public:
  TreeNode(GreasePencilLayerTreeNodeType type)
  {
    this->type = type;
    this->name = nullptr;
  }
  TreeNode(GreasePencilLayerTreeNodeType type, StringRefNull name)
  {
    this->type = type;
    this->name = BLI_strdup(name.c_str());
  }
  TreeNode(const TreeNode &other)
  {
    this->name = BLI_strdup(other.name);
  }
  TreeNode(TreeNode &&other)
  {
    this->name = other.name;
    other.name = nullptr;
  }
  TreeNode &operator=(const TreeNode &other)
  {
    if (this != &other) {
      if (this->name) {
        MEM_freeN(this->name);
      }
      this->name = BLI_strdup(other.name);
    }
    return *this;
  }
  TreeNode &operator=(TreeNode &&other)
  {
    if (this != &other) {
      this->name = other.name;
      other.name = nullptr;
    }
    return *this;
  }
  ~TreeNode()
  {
    if (this->name) {
      MEM_freeN(this->name);
    }
  }

 public:
  class PreOrderRange {
    class Iterator {
      using iterator_category = std::forward_iterator_tag;

     private:
      Stack<TreeNode *> next_node_;

     public:
      explicit Iterator(TreeNode *root)
      {
        if (root != nullptr) {
          for (auto it = root->children_.rbegin(); it != root->children_.rend(); it++) {
            next_node_.push(&(*it));
          }
        }
      }

      TreeNode &operator*()
      {
        return *next_node_.peek();
      }

      const TreeNode &operator*() const
      {
        return *next_node_.peek();
      }

      Iterator &operator++()
      {
        BLI_assert(!next_node_.is_empty());
        TreeNode &next_node = *next_node_.pop();
        for (auto it = next_node.children_.rbegin(); it != next_node.children_.rend(); it++) {
          next_node_.push(&(*it));
        }
        return *this;
      }

      Iterator operator++(int)
      {
        Iterator old = *this;
        operator++();
        return old;
      }

      bool operator==(Iterator &other) const
      {
        if (next_node_.size() == 0) {
          return other.next_node_.size() == 0;
        }
        return (next_node_.size() == other.next_node_.size()) &&
               next_node_.peek() == other.next_node_.peek();
      }

      bool operator!=(Iterator &other) const
      {
        return !(*this == other);
      }
    };

   private:
    TreeNode *_root;

   public:
    explicit PreOrderRange(TreeNode *root) : _root(root)
    {
    }

    Iterator begin()
    {
      return Iterator(_root);
    }

    Iterator end()
    {
      return Iterator(nullptr);
    }
  };

 public:
  constexpr bool is_group() const
  {
    return type == GREASE_PENCIL_LAYER_TREE_GROUP;
  }

  constexpr bool is_layer() const
  {
    return type == GREASE_PENCIL_LAYER_TREE_LEAF;
  }

  LayerGroup &as_group()
  {
    return *static_cast<LayerGroup *>(this);
  }
  Layer &as_layer()
  {
    return *static_cast<Layer *>(this);
  }

  PreOrderRange children_in_pre_order()
  {
    return PreOrderRange(this);
  }

  void foreach_children_pre_order(ItemIterFn function)
  {
    for (TreeNode &child : children_) {
      child.foreach_children_pre_order_recursive_(function);
    }
  }

  void foreach_leaf_pre_order(ItemIterFn function)
  {
    for (TreeNode &child : children_) {
      child.foreach_leaf_pre_order_recursive_(function);
    }
  }

 private:
  void foreach_children_pre_order_recursive_(ItemIterFn function)
  {
    function(*this);
    for (TreeNode &child : children_) {
      child.foreach_children_pre_order_recursive_(function);
    }
  }

  void foreach_leaf_pre_order_recursive_(ItemIterFn function)
  {
    if (children_.size() == 0) {
      function(*this);
    }
    for (TreeNode &child : children_) {
      child.foreach_children_pre_order_recursive_(function);
    }
  }
};

class Layer : public TreeNode, ::GreasePencilLayer {
 private:
  Map<int, int> frames_;

 public:
  Layer() : TreeNode(GREASE_PENCIL_LAYER_TREE_LEAF), frames_()
  {
  }
  Layer(StringRefNull name) : TreeNode(GREASE_PENCIL_LAYER_TREE_LEAF, name), frames_()
  {
  }
  Layer(const Layer &other)
      : TreeNode(GREASE_PENCIL_LAYER_TREE_LEAF, other.name), frames_(other.frames_)
  {
  }
  Layer(Layer &&other) : TreeNode(std::move(other))
  {
    frames_ = std::move(other.frames_);
  }
  Layer &operator=(const Layer &other)
  {
    return copy_assign_container(*this, other);
  }
  Layer &operator=(Layer &&other)
  {
    if (this != &other) {
      frames_ = std::move(other.frames_);
    }
    return *this;
  }

  bool operator==(const Layer &other) const
  {
    return this == &other;
  }
  bool operator!=(const Layer &other) const
  {
    return this != &other;
  }

  Layer &as_layer()
  {
    return *this;
  }
};

class LayerGroup : public TreeNode {
 public:
  LayerGroup() : TreeNode(GREASE_PENCIL_LAYER_TREE_GROUP)
  {
  }
  LayerGroup(StringRefNull name) : TreeNode(GREASE_PENCIL_LAYER_TREE_GROUP, name)
  {
  }
  LayerGroup(const LayerGroup &other) : TreeNode(GREASE_PENCIL_LAYER_TREE_GROUP, other.name)
  {
  }
  LayerGroup(LayerGroup &&other) : TreeNode(std::move(other))
  {
  }
  ~LayerGroup()
  {
  }

 public:
  bool operator==(const LayerGroup &other) const
  {
    return this == &other;
  }
  bool operator!=(const LayerGroup &other) const
  {
    return this != &other;
  }

  void add_group(LayerGroup &group)
  {
    children_.append(group);
  }

  void add_group(LayerGroup &&group)
  {
    children_.append(std::move(group));
  }

  void add_layer(Layer &layer)
  {
    children_.append(layer);
  }

  void add_layer(Layer &&layer)
  {
    children_.append(std::move(layer));
  }

  int num_children()
  {
    return children_.size();
  }

  void remove_child(int64_t index)
  {
    BLI_assert(index >= 0 && index < children_.size());
    children_.remove(index);
  }
};

class LayerTree {
 private:
  LayerGroup root_;
};

namespace convert {

CurvesGeometry legacy_gpencil_frame_to_curves_geometry(bGPDframe &gpf);

void legacy_gpencil_to_grease_pencil(bGPdata &gpd, GreasePencil &grease_pencil);

}  // namespace convert

}  // namespace gpencil

class GreasePencilRuntime {
 private:
  gpencil::LayerTree layer_tree_;
};

}  // namespace blender::bke

struct Main;
struct BoundBox;

void *BKE_grease_pencil_add(Main *bmain, const char *name);
BoundBox *BKE_grease_pencil_boundbox_get(Object *ob);
