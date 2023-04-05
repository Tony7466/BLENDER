/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup DNA
 */

#pragma once

#include "DNA_ID.h"
#include "DNA_curves_types.h"

#ifdef __cplusplus
#  include "BLI_function_ref.hh"
#  include "BLI_map.hh"
#  include "BLI_math_vector_types.hh"
#  include "BLI_span.hh"
namespace blender::bke {
class GreasePencilRuntime;
class GreasePencilDrawingRuntime;
struct StrokePoint;
namespace gpencil {
class Layer;
class LayerGroup;
}
}  // namespace blender::bke
using GreasePencilRuntimeHandle = blender::bke::GreasePencilRuntime;
using GreasePencilDrawingRuntimeHandle = blender::bke::GreasePencilDrawingRuntime;
#else
typedef struct GreasePencilRuntimeHandle GreasePencilRuntimeHandle;
typedef struct GreasePencilDrawingRuntimeHandle GreasePencilDrawingRuntimeHandle;
#endif

#ifdef __cplusplus
extern "C" {
#endif

struct GreasePencil;
struct BlendDataReader;
struct BlendWriter;

typedef enum GreasePencilDrawingType {
  GREASE_PENCIL_DRAWING = 0,
  GREASE_PENCIL_DRAWING_REFERENCE = 1,
} GreasePencilLayerType;

typedef struct GreasePencilDrawingOrReference {
  /**
   * One of `GreasePencilDrawingType`.
   * Indicates if this is an actual drawing or a drawing referenced from another object.
   */
  uint8_t type;
  char _pad[3];
  /**
   * Flag. Used to set e.g. the selection status.
   */
  uint32_t flag;
} GreasePencilDrawingOrReference;

/**
 * A grease pencil drawing is a set of strokes. The data is stored using the `CurvesGeometry` data
 * structure and the custom attributes within it.
 */
typedef struct GreasePencilDrawing {
  GreasePencilDrawingOrReference base;
  /**
   * The stroke data for this drawing.
   */
  CurvesGeometry geometry;
#ifdef __cplusplus
  /**
   * The triangles for all the fills in the geometry.
   */
  blender::Span<blender::uint3> triangles() const;
  void tag_positions_changed();
  bool has_stroke_buffer();
  blender::Span<blender::bke::StrokePoint> stroke_buffer();
#endif
  /**
   * Runtime data on the drawing.
   */
  GreasePencilDrawingRuntimeHandle *runtime;
} GreasePencilDrawing;

typedef struct GreasePencilDrawingReference {
  GreasePencilDrawingOrReference base;
  /**
   * A reference to another GreasePencil data-block.
   * If the data-block has multiple drawings, this drawing references all of them sequentially.
   * See the note in `GreasePencilLayer->frames()` for a detailed explanation of this.
   */
  struct GreasePencil *id_reference;
} GreasePencilDrawingReference;

/**
 * Storage for the Map in `blender::bke::gpencil::Layer`.
 * See the description there for more detail.
 */
typedef struct GreasePencilLayerFramesMapStorage {
  /* Array of `frames` keys (sorted in ascending order). */
  int *keys;
  /* Array of `frames` values (order matches the keys array). */
  int *values;
  /* Size of the map (number of key-value pairs). */
  int size;
  char _pad[4];
} GreasePencilLayerFramesMapStorage;

/**
 * A grease pencil layer is a collection of drawings mapped to a specific time on the timeline.
 */
typedef struct GreasePencilLayer {
  /* Only used for storage in the .blend file. */
  GreasePencilLayerFramesMapStorage frames_storage;
} GreasePencilLayer;

typedef enum GreasePencilLayerTreeNodeType {
  GREASE_PENCIL_LAYER_TREE_LEAF = 0,
  GREASE_PENCIL_LAYER_TREE_GROUP = 1,
} GreasePencilLayerTreeNodeType;

typedef struct GreasePencilLayerTreeNode {
  /**
   * One of `GreasePencilLayerTreeNodeType`.
   * Indicates the type of struct this element is.
   */
  uint8_t type;

  /**
   * Color tag.
   */
  uint8_t color[3];

  /**
   * Flag. Used to set e.g. the selection, visibility, ... status.
   */
  uint32_t flag;

  /**
   * Name of the layer/group. Dynamic length.
   */
  char *name;
} GreasePencilLayerTreeNode;

typedef struct GreasePencilLayerTreeGroup {
  GreasePencilLayerTreeNode base;
  int children_num;
  char _pad[4];
} GreasePencilLayerTreeGroup;

typedef struct GreasePencilLayerTreeLeaf {
  GreasePencilLayerTreeNode base;
  GreasePencilLayer layer;
} GreasePencilLayerTreeLeaf;

/* Only used for storage in the .blend file. */
typedef struct GreasePencilLayerTreeStorage {
  /* Array of tree nodes. Pre-order serialization of the layer tree. */
  GreasePencilLayerTreeNode **nodes;
  int nodes_num;
  char _pad[4];
} GreasePencilLayerTreeStorage;

/**
 * The grease pencil data-block.
 * It holds a set of grease pencil drawings, a tree of layer groups, and a set of layers whithin
 * each layer group.
 */
typedef struct GreasePencil {
  ID id;
  /** Animation data. */
  struct AnimData *adt;

  /**
   * An array of GreasePencilDrawing's.
   */
  GreasePencilDrawingOrReference **drawing_array;
  int drawing_array_size;
  char _pad[4];

#ifdef __cplusplus
  blender::Span<GreasePencilDrawingOrReference *> drawings() const;
  void foreach_visible_drawing(int frame,
                               blender::FunctionRef<void(GreasePencilDrawing &)> function);
  void read_drawing_array(BlendDataReader *reader);
  void write_drawing_array(BlendWriter *writer);
  void free_drawing_array();
#endif
  /* Only used for storage in the .blend file. */
  GreasePencilLayerTreeStorage layer_tree_storage;
#ifdef __cplusplus
  void save_layer_tree_to_storage();
  void load_layer_tree_from_storage();
  void read_layer_tree_storage(BlendDataReader *reader);
  void write_layer_tree_storage(BlendWriter *writer);
  void free_layer_tree_storage();
#endif
  /**
   * An array of materials.
   */
  struct Material **material_array;
  short material_array_size;
  char _pad2[2];

  /**
   * Global flag on the data-block.
   */
  uint32_t flag;

  /**
   * Runtime struct pointer.
   */
  GreasePencilRuntimeHandle *runtime;
#ifdef __cplusplus
  blender::bke::gpencil::Layer *get_active_layer();
  blender::bke::gpencil::LayerGroup &root_group();
#endif
} GreasePencil;

#ifdef __cplusplus
}
#endif
