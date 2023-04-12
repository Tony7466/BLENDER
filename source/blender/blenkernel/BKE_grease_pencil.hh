/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bke
 * \brief Low-level operations for grease pencil.
 */

#include "BLI_function_ref.hh"
#include "BLI_map.hh"
#include "BLI_math_vector.h"
#include "BLI_math_vector_types.hh"
#include "BLI_shared_cache.hh"
#include "BLI_stack.hh"
#include "BLI_string.h"

#include "DNA_gpencil_legacy_types.h"
#include "DNA_grease_pencil_types.h"

namespace blender::bke {

namespace gpencil {

class LayerGroup;
class Layer;

/**
 * A TreeNode represents one node in the layer tree.
 * It can either be a layer or a group. The node has zero children if it is a layer or zero or more
 * children if it is a group.
 * This class is mainly used for iteration over the layer tree.
 */
class TreeNode : public ::GreasePencilLayerTreeNode {
  using TreeNodeIterFn = FunctionRef<void(TreeNode &)>;
  using LayerIterFn = FunctionRef<void(Layer &)>;

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
  TreeNode(const TreeNode &other) : TreeNode(GreasePencilLayerTreeNodeType(other.type))
  {
    if (other.name) {
      this->name = BLI_strdup(other.name);
    }
  }
  TreeNode(TreeNode &&other)
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
    explicit PreOrderRange(TreeNode *root) : _root(root) {}

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

  int total_num_children() const
  {
    int total = 0;
    Stack<TreeNode *> stack;
    for (auto it = this->children_.rbegin(); it != this->children_.rend(); it++) {
      stack.push((*it).get());
    }
    while (!stack.is_empty()) {
      TreeNode &next_node = *stack.pop();
      total++;
      for (auto it = next_node.children_.rbegin(); it != next_node.children_.rend(); it++) {
        stack.push((*it).get());
      }
    }
    return total;
  }

  PreOrderRange children_in_pre_order()
  {
    return PreOrderRange(this);
  }

  void foreach_children_pre_order(TreeNodeIterFn function)
  {
    for (auto &child : children_) {
      child->foreach_children_pre_order_recursive_(function);
    }
  }

  void foreach_layer_pre_order(LayerIterFn function)
  {
    for (auto &child : children_) {
      child->foreach_layer_pre_order_recursive_(function);
    }
  }

 private:
  void foreach_children_pre_order_recursive_(TreeNodeIterFn function)
  {
    function(*this);
    for (auto &child : children_) {
      child->foreach_children_pre_order_recursive_(function);
    }
  }

  void foreach_layer_pre_order_recursive_(LayerIterFn function)
  {
    if (this->is_layer()) {
      function(this->as_layer());
    }
    for (auto &child : children_) {
      child->foreach_layer_pre_order_recursive_(function);
    }
  }
};

/**
 * A layer maps drawings to scene frames. It can be thought of as one independent channel in the
 * timeline.
 */
class Layer : public TreeNode, ::GreasePencilLayer {
 private:
  /**
   * This Map maps a scene frame number (key) to an index into GreasePencil->drawings (value). The
   * frame number indicates the first frame the drawing is shown. The end time is implicitly
   * defined by the next greater frame number (key) in the map. If the value mapped to (index) is
   * -1, no drawing is shown at this frame.
   *
   *    \example:
   *
   *    {0: 0, 5: 1, 10: -1, 12: 2, 16: -1}
   *
   *    In this example there are three drawings (drawing #0, drawing #1 and drawing #2). The first
   *    drawing starts at frame 0 and ends at frame 5 (excusive). The second drawing starts at
   *    frame 5 and ends at frame 10. Finally, the third drawing starts at frame 12 and ends at
   *    frame 16.
   *
   *           | | | | | | | | | | |1|1|1|1|1|1|1|
   *    Time:  |0|1|2|3|4|5|6|7|8|9|0|1|2|3|4|5|6|...
   *    Frame: [#0      ][#1      ]    [#2    ]
   *
   * \note If a drawing references another data-block, all of the drawings in that data-block are
   * mapped sequentially to the frames (frame-by-frame). If another frame starts, the rest of the
   * referenced drawings are discarded. If the frame is longer than the number of referenced
   * drawings, then the last referenced drawing is held for the rest of the duration.
   */
  Map<int, int> frames_;
  mutable SharedCache<Vector<int>> sorted_keys_cache_;

 public:
  Layer() : TreeNode(GREASE_PENCIL_LAYER_TREE_LEAF), frames_() {}
  Layer(StringRefNull name) : TreeNode(GREASE_PENCIL_LAYER_TREE_LEAF, name), frames_() {}
  Layer(const Layer &other) : TreeNode(other)
  {
    frames_ = other.frames_;
  }
  Layer(Layer &&other) : TreeNode(std::move(other))
  {
    frames_ = std::move(other.frames_);
  }
  ~Layer() {}

  bool operator==(const Layer &other) const
  {
    return this == &other;
  }
  bool operator!=(const Layer &other) const
  {
    return this != &other;
  }

  const Map<int, int> &frames()
  {
    return frames_;
  }

  Map<int, int> &frames_for_write()
  {
    sorted_keys_cache_.tag_dirty();
    return frames_;
  }

  bool insert_frame(int frame_number, int index)
  {
    sorted_keys_cache_.tag_dirty();
    return frames_for_write().add(frame_number, index);
  }

  bool overwrite_frame(int frame_number, int index)
  {
    sorted_keys_cache_.tag_dirty();
    return frames_for_write().add_overwrite(frame_number, index);
  }

  Span<int> sorted_keys()
  {
    sorted_keys_cache_.ensure([&](Vector<int> &r_data) {
      r_data.clear_and_shrink();
      r_data.reserve(frames().size());
      for (int64_t key : frames().keys()) {
        r_data.append(key);
      }
      std::sort(r_data.begin(), r_data.end());
    });
    return sorted_keys_cache_.data().as_span();
  }

  void load_from_storage(GreasePencilLayer &node)
  {
    for (int i = 0; i < node.frames_storage.size; i++) {
      this->frames_.add(node.frames_storage.keys[i], node.frames_storage.values[i]);
    }
  }

  /**
   * Return the index of the drawing at frame \a frame or -1 if there is no drawing.
   */
  int drawing_at(int frame)
  {
    Span<int> sorted_keys = this->sorted_keys();
    /* Before the first drawing, return no drawing. */
    if (frame < sorted_keys.first()) {
      return -1;
    }
    /* After or at the last drawing, return the last drawing. */
    if (frame >= sorted_keys.last()) {
      return frames().lookup(sorted_keys.last());
    }
    /* Search for the drawing. upper_bound will get the drawing just after. */
    auto it = std::upper_bound(sorted_keys.begin(), sorted_keys.end(), frame);
    if (it == sorted_keys.end() || it == sorted_keys.begin()) {
      return -1;
    }
    return frames().lookup(*std::prev(it));
  }
};

/**
 * A LayerGroup is a grouping of zero or more Layers.
 */
class LayerGroup : public TreeNode {
 public:
  LayerGroup() : TreeNode(GREASE_PENCIL_LAYER_TREE_GROUP) {}
  LayerGroup(StringRefNull name) : TreeNode(GREASE_PENCIL_LAYER_TREE_GROUP, name) {}
  LayerGroup(const LayerGroup &other) : TreeNode(other)
  {
    children_.reserve(other.children_.size());
    for (const std::unique_ptr<TreeNode> &elem : other.children_) {
      if (elem.get()->is_group()) {
        children_.append(std::make_unique<LayerGroup>(elem.get()->as_group()));
      }
      else if (elem.get()->is_layer()) {
        children_.append(std::make_unique<Layer>(elem.get()->as_layer()));
      }
    }
  }
  LayerGroup(LayerGroup &&other) : TreeNode(std::move(other))
  {
    /* TODO! */
    // children_.reserve(other.children_.size());
    // for (const std::unique_ptr<TreeNode> &elem : other.children_) {
    //   if (elem.get()->is_group()) {
    //     children_.append_as(std::move(*elem));
    //   }
    //   else if (elem.get()->is_layer()) {
    //     children_.append_as(std::move(*elem));
    //   }
    // }
    // other.children_.clear();
  }
  ~LayerGroup() {}

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

  Layer &add_layer(Layer &layer)
  {
    int64_t index = children_.append_and_get_index(std::make_unique<Layer>(layer));
    return children_[index].get()->as_layer();
  }

  Layer &add_layer(Layer &&layer)
  {
    int64_t index = children_.append_and_get_index(std::make_unique<Layer>(layer));
    return children_[index].get()->as_layer();
  }

  int num_children() const
  {
    return children_.size();
  }

  void remove_child(int64_t index)
  {
    BLI_assert(index >= 0 && index < children_.size());
    children_.remove(index);
  }
};

namespace convert {

void legacy_gpencil_frame_to_curves_geometry(GreasePencilDrawing &drawing, bGPDframe &gpf);

void legacy_gpencil_to_grease_pencil(GreasePencil &grease_pencil, bGPdata &gpd);

}  // namespace convert

}  // namespace gpencil

using namespace blender::bke::gpencil;

struct StrokePoint {
  float3 position;
  float radius;
  float opacity;
  float4 color;
};

class GreasePencilDrawingRuntime {
 public:
  mutable SharedCache<Vector<uint3>> triangles_cache;

  /**
   * Stroke cache for a stroke that is currently being drawn.
   */
  Vector<StrokePoint> stroke_cache = {};
  Vector<uint3> stroke_triangle_cache = {};
  int stroke_mat = 0;
};

class GreasePencilRuntime {
 private:
  LayerGroup root_group_;

 public:
  GreasePencilRuntime() {}
  GreasePencilRuntime(const GreasePencilRuntime &other) : root_group_(other.root_group_) {}

  LayerGroup &root_group()
  {
    return root_group_;
  }

 public:
  void *batch_cache = nullptr;
};

}  // namespace blender::bke

struct Main;
struct Depsgraph;
struct BoundBox;
struct Scene;
struct Object;

void *BKE_grease_pencil_add(Main *bmain, const char *name);
GreasePencil *BKE_grease_pencil_new_nomain();
BoundBox *BKE_grease_pencil_boundbox_get(Object *ob);
void BKE_grease_pencil_data_update(struct Depsgraph *depsgraph,
                                   struct Scene *scene,
                                   struct Object *object);
