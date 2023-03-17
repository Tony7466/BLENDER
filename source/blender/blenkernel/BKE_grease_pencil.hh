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
  Vector<std::unique_ptr<TreeNode>> children_;

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
    this->type = other.type;
    this->name = BLI_strdup(other.name);
    children_.reserve(other.children_.size());
    for (const std::unique_ptr<TreeNode> &elem : other.children_) {
      children_.append(std::make_unique<TreeNode>(*elem));
    }
  }
  TreeNode(TreeNode &&other) : children_(std::move(other.children_))
  {
    this->name = other.name;
    other.name = nullptr;
  }
  TreeNode &operator=(const TreeNode &other) = delete;
  TreeNode &operator=(TreeNode &&other) = delete;
  virtual ~TreeNode()
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
            next_node_.push((*it).get());
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
          next_node_.push((*it).get());
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
    return this->type == GREASE_PENCIL_LAYER_TREE_GROUP;
  }

  constexpr bool is_layer() const
  {
    return this->type == GREASE_PENCIL_LAYER_TREE_LEAF;
  }

  LayerGroup &as_group();
  Layer &as_layer();

  PreOrderRange children_in_pre_order()
  {
    return PreOrderRange(this);
  }

  void foreach_children_pre_order(ItemIterFn function)
  {
    for (auto &child : children_) {
      child->foreach_children_pre_order_recursive_(function);
    }
  }

  void foreach_leaf_pre_order(ItemIterFn function)
  {
    for (auto &child : children_) {
      child->foreach_leaf_pre_order_recursive_(function);
    }
  }

 private:
  void foreach_children_pre_order_recursive_(ItemIterFn function)
  {
    function(*this);
    for (auto &child : children_) {
      child->foreach_children_pre_order_recursive_(function);
    }
  }

  void foreach_leaf_pre_order_recursive_(ItemIterFn function)
  {
    if (children_.size() == 0) {
      function(*this);
    }
    for (auto &child : children_) {
      child->foreach_children_pre_order_recursive_(function);
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
  Layer(const Layer &other) : TreeNode(other)
  {
  }
  Layer(Layer &&other) : TreeNode(std::move(other))
  {
    frames_ = std::move(other.frames_);
  }
  ~Layer()
  {
  }

  bool operator==(const Layer &other) const
  {
    return this == &other;
  }
  bool operator!=(const Layer &other) const
  {
    return this != &other;
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
  LayerGroup(const LayerGroup &other) : TreeNode(other)
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
    children_.append(std::make_unique<LayerGroup>(group));
  }

  void add_group(LayerGroup &&group)
  {
    children_.append(std::make_unique<LayerGroup>(group));
  }

  void add_layer(Layer &layer)
  {
    children_.append(std::make_unique<Layer>(layer));
  }

  void add_layer(Layer &&layer)
  {
    children_.append(std::make_unique<Layer>(layer));
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
