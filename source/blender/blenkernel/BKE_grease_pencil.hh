/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation. */

#pragma once

/** \file
 * \ingroup bke
 * \brief Low-level operations for grease pencil.
 */

#include "BLI_function_ref.hh"
#include "BLI_map.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_shared_cache.hh"
#include "BLI_utility_mixins.hh"

#include "DNA_gpencil_legacy_types.h"
#include "DNA_grease_pencil_types.h"

namespace blender::bke {

namespace greasepencil {

class LayerGroup;
class Layer;

/**
 * A TreeNode represents one node in the layer tree.
 * It can either be a layer or a group. The node has zero children if it is a layer or zero or more
 * children if it is a group.
 */
class TreeNode : public ::GreasePencilLayerTreeNode, NonMovable {
  friend class LayerGroup;

 public:
  explicit TreeNode(GreasePencilLayerTreeNodeType type);
  explicit TreeNode(GreasePencilLayerTreeNodeType type, StringRefNull name);
  TreeNode(const TreeNode &other);
  TreeNode &operator=(const TreeNode &other) = delete;
  virtual ~TreeNode();

 private:
  LayerGroup *parent_ = nullptr;
  Vector<std::unique_ptr<TreeNode>> children_;

 public:
  /**
   * \returns true if this node is a LayerGroup.
   */
  bool is_group() const
  {
    return this->type == GP_LAYER_TREE_GROUP;
  }

  /**
   * \returns true if this node is a Layer.
   */
  bool is_layer() const
  {
    return this->type == GP_LAYER_TREE_LEAF;
  }

  /**
   * \returns this tree node as a LayerGroup.
   * \note This results in undefined behavior if the node is not a LayerGroup.
   */
  const LayerGroup &as_group() const;

  /**
   * \returns this tree node as a Layer.
   * \note This results in undefined behavior if the node is not a Layer.
   */
  const Layer &as_layer() const;

  /**
   * \returns this tree node as a mutable LayerGroup.
   * \note This results in undefined behavior if the node is not a LayerGroup.
   */
  LayerGroup &as_group_for_write();

  /**
   * \returns this tree node as a mutable Layer.
   * \note This results in undefined behavior if the node is not a Layer.
   */
  Layer &as_layer_for_write();
};

/**
 * A layer mask stores a reference to a layer that will mask other layers.
 */
class LayerMask : public ::GreasePencilLayerMask {
 public:
  LayerMask();
  explicit LayerMask(StringRefNull name);
  LayerMask(const LayerMask &other);
  ~LayerMask();
};

/**
 * A layer maps drawings to scene frames. It can be thought of as one independent channel in the
 * timeline.
 */
class Layer : public TreeNode, public ::GreasePencilLayer {
 private:
  /**
   * This Map maps a scene frame number (key) to a GreasePencilFrame. This struct holds an index
   * (drawing_index) to the drawing in the GreasePencil->drawings array. The frame number indicates
   * the first frame the drawing is shown. The end time is implicitly defined by the next greater
   * frame number (key) in the map. If the value mapped to (index) is -1, no drawing is shown at
   * this frame.
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
   *                  | | | | | | | | | | |1|1|1|1|1|1|1|
   *    Scene Frame:  |0|1|2|3|4|5|6|7|8|9|0|1|2|3|4|5|6|...
   *    Drawing:      [#0      ][#1      ]    [#2    ]
   *
   * \note If a drawing references another data-block, all of the drawings in that data-block are
   * mapped sequentially to the frames (frame-by-frame). If another frame starts, the rest of the
   * referenced drawings are discarded. If the frame is longer than the number of referenced
   * drawings, then the last referenced drawing is held for the rest of the duration.
   */
  Map<int, GreasePencilFrame> frames_;
  /**
   * Caches a sorted vector of the keys of `frames_`.
   */
  mutable SharedCache<Vector<int>> sorted_keys_cache_;
  /**
   * A vector of LayerMask. This layer will be masked by the layers referenced in the masks.
   * A layer can have zero or more layer masks.
   */
  Vector<LayerMask> masks_;

 public:
  Layer();
  explicit Layer(StringRefNull name);
  Layer(const Layer &other);
  ~Layer();

  /**
   * \returns the frames mapping.
   */
  const Map<int, GreasePencilFrame> &frames() const;
  Map<int, GreasePencilFrame> &frames_for_write();

  /**
   * \returns the layer masks.
   */
  Span<LayerMask> masks() const;
  Vector<LayerMask> &masks_for_write();

  bool is_visible() const;
  bool is_locked() const;

  /**
   * Inserts the frame into the layer. Fails if there exists a frame at \a frame_number already.
   * \returns true on success.
   */
  bool insert_frame(int frame_number, const GreasePencilFrame &frame);
  bool insert_frame(int frame_number, GreasePencilFrame &&frame);

  /**
   * Inserts the frame into the layer. If there exists a frame at \a frame_number already, it is
   * overwritten.
   * \returns true on success.
   */
  bool overwrite_frame(int frame_number, const GreasePencilFrame &frame);
  bool overwrite_frame(int frame_number, GreasePencilFrame &&frame);

  /**
   * Returns the sorted (start) frame numbers of the frames of this layer.
   * \note This will cache the keys lazily.
   */
  Span<int> sorted_keys() const;

  /**
   * \returns the index of the drawing at frame \a frame or -1 if there is no drawing.
   */
  int drawing_index_at(int frame) const;

  /**
   * Should be called whenever the keys in the frames map have changed.
   */
  void tag_frames_map_keys_changed();
};

/**
 * A LayerGroup is a grouping of zero or more Layers.
 */
class LayerGroup : public TreeNode {
 public:
  LayerGroup() : TreeNode(GP_LAYER_TREE_GROUP) {}
  explicit LayerGroup(const StringRefNull name) : TreeNode(GP_LAYER_TREE_GROUP, name) {}
  LayerGroup(const LayerGroup &other);

 private:
  /**
   * CacheMutex for `children_cache_` and `layer_cache_`;
   */
  mutable CacheMutex children_cache_mutex_;
  /**
   * Caches all the children of this group in a single pre-order vector.
   */
  mutable Vector<TreeNode *> children_cache_;
  /**
   * Caches all the layers in this group in a single pre-order vector.
   */
  mutable Vector<Layer *> layer_cache_;

 public:
  /**
   * Adds a group at the end of this group.
   */
  void add_group(LayerGroup &group);
  void add_group(LayerGroup &&group);

  /**
   * Adds a layer at the end of this group and returns it.
   */
  Layer &add_layer(Layer &layer);
  Layer &add_layer(Layer &&layer);

  /**
   * Returns the number of direct children in this group.
   */
  int64_t num_direct_children() const;

  /**
   * Returns the total number of children in this group.
   */
  int64_t num_children_total() const;

  /**
   * Removes a child from the group by index.
   */
  void remove_child(int64_t index);

  /**
   * Returns a `Span` of pointers to all the `TreeNode`s in this group.
   */
  Span<const TreeNode *> children() const;
  Span<TreeNode *> children_for_write();

  /**
   * Returns a `Span` of pointers to all the `Layers`s in this group.
   */
  Span<const Layer *> layers() const;
  Span<Layer *> layers_for_write();

  /**
   * Print the children. For debugging purposes.
   */
  void print_children(StringRefNull header) const;

 private:
  void ensure_children_cache() const;
  void tag_children_cache_dirty() const;
};

namespace convert {

void legacy_gpencil_frame_to_grease_pencil_drawing(const bGPDframe &gpf, GreasePencilDrawing &r_drawing);
void legacy_gpencil_to_grease_pencil(Main &main, GreasePencil &grease_pencil, bGPdata &gpd);

}  // namespace convert

}  // namespace greasepencil

/**
 * A single point for a stroke that is currently being drawn.
 */
struct StrokePoint {
  float3 position;
  float radius;
  float opacity;
  float4 color;
};

/**
 * Stroke cache for a stroke that is currently being drawn.
 */
struct StrokeCache {
  Vector<StrokePoint> points;
  Vector<uint3> triangles;
  int mat = 0;

  void clear()
  {
    this->points.clear_and_shrink();
    this->triangles.clear_and_shrink();
    this->mat = 0;
  }
};

class GreasePencilDrawingRuntime {
 public:
  /**
   * Triangle cache for all the strokes in the drawing.
   */
  mutable SharedCache<Vector<uint3>> triangles_cache;

  StrokeCache stroke_cache;
};

class GreasePencilRuntime {
 public:
  void *batch_cache = nullptr;

 private:
  greasepencil::LayerGroup root_group_;
  int active_layer_index_ = -1;

 public:
  GreasePencilRuntime() {}
  GreasePencilRuntime(const GreasePencilRuntime &other);

 public:
  const greasepencil::LayerGroup &root_group() const;
  greasepencil::LayerGroup &root_group_for_write();

  Span<const greasepencil::Layer *> layers() const;
  Span<greasepencil::Layer *> layers_for_write();

  bool has_active_layer() const;
  const greasepencil::Layer &active_layer() const;
  greasepencil::Layer &active_layer_for_write();
  void set_active_layer_index(int index);
  int active_layer_index() const;
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
