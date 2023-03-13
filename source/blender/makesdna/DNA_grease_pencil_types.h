/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup DNA
 */

#pragma once

#include "DNA_ID.h"
#include "DNA_curves_types.h"

#ifdef __cplusplus
#  include "BLI_map.hh"
#  include "BLI_span.hh"
namespace blender::bke {
class GreasePencilLayerRuntime;
class GreasePencilRuntime;
}  // namespace blender::bke
using GreasePencilLayerRuntimeHandle = blender::bke::GreasePencilLayerRuntime;
using GreasePencilRuntimeHandle = blender::bke::GreasePencilRuntime;
#else
typedef struct GreasePencilLayerRuntimeHandle GreasePencilLayerRuntimeHandle;
typedef struct GreasePencilRuntimeHandle GreasePencilRuntimeHandle;
#endif

#ifdef __cplusplus
extern "C" {
#endif

struct GreasePencil;

typedef enum GreasePencilDrawingType {
  GREASE_PENCIL_DRAWING = 0,
  GREASE_PENCIL_DRAWING_REFERENCE = 1,
} GreasePencilLayerType;

typedef struct GreasePencilDrawingOrReference {
  /**
   * One of `GreasePencilDrawingType`.
   * Indicates if this is an actual drawing or a drawing referenced from another object.
   */
  char type;
  /**
   * Flag. Used to set e.g. the selection status.
   */
  int flag;
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
} GreasePencilDrawing;

typedef struct GreasePencilDrawingReference {
  GreasePencilDrawingOrReference base;
  /**
   * A reference to another GreasePencil data-block.
   * If the data-block has multiple drawings, this drawing references
   * all of them sequentially.
   */
  struct GreasePencil *id_reference;
} GreasePencilDrawingReference;

/**
 * A grease pencil layer is a collection of drawings mapped to a specific time on the timeline.
 */
typedef struct GreasePencilLayer {
  /**
   * This Map maps a scene frame number (key) to an index into
   * GreasePencil->drawings (value). The frame number indicates
   * the first frame the drawing is shown. The end time is implicitly
   * defined by the next greater frame number (key) in the map.
   * If the value mapped to (index) is -1, no drawing is shown
   * at this frame.
   *
   *    Example:
   *
   *    {0: 0, 5: 1, 10: -1, 12: 2, 16: -1}
   *
   *    In this example there are three drawings (drawing #0,
   *    drawing #1 and drawing #2). The first drawing starts at frame 0
   *    and ends at frame 5 (excusive). The second drawing starts at
   *    frame 5 and ends at frame 10. Finally, the third drawing starts
   *    at frame 12 and ends at frame 16.
   *
   *           | | | | | | | | | | |1|1|1|1|1|1|1|
   *    Time:  |0|1|2|3|4|5|6|7|8|9|0|1|2|3|4|5|6|...
   *    Frame: [#0      ][#1      ]    [#2    ]
   *
   * Note: If a drawing references another data-block, all of the drawings
   *       in that data-block are mapped sequentially to the frames.
   *       If another frame starts, the rest of the mapped drawings
   *       are discarded.
   */
#ifdef __cplusplus
  const blender::Map<int, int> &frames() const;
#endif

  /* Only used for storage in the .blend file. */
  struct {
    /* Array of `frames` keys (sorted in ascending order). */
    int *keys;
    int keys_num;

    /* Array of `frames` values (order matches the keys array). */
    int *values;
    int values_num;
  } frames_storage;

  /**
   * Runtime struct pointer.
   */
  GreasePencilLayerRuntimeHandle *runtime;
} GreasePencilLayer;

typedef enum GreasePencilLayerTreeNodeType {
  GREASE_PENCIL_LAYER_TREE_LEAF = 0,
  GREASE_PENCIL_LAYER_TREE_GROUP = 1,
} GreasePencilLayerTreeNode;

typedef struct GreasePencilLayerTreeNode {
  /**
   * One of `GreasePencilLayerTreeNodeType`.
   * Indicates the type of struct this element is.
   */
  char type;

  /**
   * Name of the layer/group. Dynamic length.
   */
  char *name;

  /**
   * Flag. Used to set e.g. the selection, visibility, ... status.
   */
  int flag;

  /**
   * Color tag.
   */
  uchar color[3];
} GreasePencilLayerTreeNode;

typedef struct GreasePencilLayerTreeGroup {
  GreasePencilLayerTreeNode base;
  int children_num;
} GreasePencilLayerTreeGroup;

typedef struct GreasePencilLayerTreeLeaf {
  GreasePencilLayerTreeNode base;
  GreasePencilLayer layer;
} GreasePencilLayerTreeLeaf;

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
  GreasePencilDrawingOrReference *drawing_array;
  int drawing_array_num;
#ifdef __cplusplus
  blender::Span<GreasePencilDrawingOrReference> drawings() const;
#endif

  /**
   * The layer tree.
   */
#ifdef __cplusplus
  // const bke::gpencil::LayerTree &layer_tree() const;
#endif

  /* Only used for storage in the .blend file. */
  struct {
    /* Array of tree nodes. Pre-order serialization of the layer tree. */
    GreasePencilLayerTreeNode **nodes;
    int nodes_num;
  } layer_tree_storage;

  /**
   * An array of materials.
   */
  struct Material **material_array;
  int material_array_size;

  /**
   * Global flag on the data-block.
   */
  int flag;

  /**
   * Runtime struct pointer.
   */
  GreasePencilRuntimeHandle *runtime;
} GreasePencil;

#ifdef __cplusplus
}
#endif
