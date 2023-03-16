/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bke
 * \brief Low-level operations for grease pencil.
 */

#include "BLI_map.hh"
#include "BLI_stack.hh"
#include "BLI_string.h"

#include "DNA_gpencil_legacy_types.h"
#include "DNA_grease_pencil_types.h"

namespace blender::bke {

class GreasePencilLayerRuntime {
 public:
  Map<int, int> frames;
};

namespace gpencil {

class TreeNode : public ::GreasePencilLayerTreeNode {
 private:
  Vector<TreeNode, 0> children_;

 public:
  TreeNode()
  {
    this->name = nullptr;
  }
  TreeNode(StringRefNull name)
  {
    this->name = BLI_strdup(name.c_str());
  }
  TreeNode(const TreeNode &other) : TreeNode(other.name)
  {
  }
  TreeNode(TreeNode &&other)
  {
    std::swap(this->name, other.name);
    other.name = nullptr;
  }
  TreeNode &operator=(const TreeNode &other)
  {
    if (this != &other) {
      this->name = BLI_strdup(other.name);
    }
    return *this;
  }
  TreeNode &operator=(TreeNode &&other)
  {
    if (this != &other) {
      std::swap(this->name, other.name);
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
  bool operator==(const TreeNode &other) const
  {
    return this == &other;
  }
  bool operator!=(const TreeNode &other) const
  {
    return this != &other;
  }

  void add_child(TreeNode &node)
  {
    children_.append(node);
  }

  void add_child(TreeNode &&node)
  {
    children_.append(std::move(node));
  }

  int size()
  {
    return children_.size();
  }

  bool remove_child(TreeNode &node)
  {
    BLI_assert(children_.size() != 0);
    int64_t index = children_.first_index_of_try(node);
    if (index < 0) {
      return false;
    }
    children_.remove(index);
    return true;
  }

  PreOrderRange children_in_pre_order()
  {
    return PreOrderRange(this);
  }
};

class Layer : TreeNode, ::GreasePencilLayer {};

namespace convert {

CurvesGeometry legacy_gpencil_frame_to_curves_geometry(bGPDframe &gpf);

void legacy_gpencil_to_grease_pencil(bGPdata &gpd, GreasePencil &grease_pencil);

}  // namespace convert

}  // namespace gpencil

}  // namespace blender::bke

struct Main;
struct BoundBox;

void *BKE_grease_pencil_add(Main *bmain, const char *name);
BoundBox *BKE_grease_pencil_boundbox_get(Object *ob);

inline const blender::Map<int, int> &GreasePencilLayer::frames() const
{
  return this->runtime->frames;
}
