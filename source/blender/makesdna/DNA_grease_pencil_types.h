/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup DNA
 */

#pragma once

#include "DNA_ID.h"
#include "DNA_curves_types.h"

#ifdef __cplusplus
#  include "BLI_map.hh"
namespace blender::bke {
class GreasePencilLayerRuntime;
}  // namespace blender::bke
using GreasePencilLayerRuntimeHandle = blender::bke::GreasePencilLayerRuntime;
#else
typedef struct GreasePencilLayerRuntimeHandle GreasePencilLayerRuntimeHandle;
#endif
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
  char type;
  /**
   * Flag. Used to set e.g. the selection status.
   */
  int flag;
} GreasePencilDrawingOrReference;

/**
 * A grease pencil drawing is a set of strokes. The data is stored using
 * the CurvesGeometry data structure and the custom attributes within it.
 * It can either own the data or reference it from another GreasePencil
 * data-block.
 *
 * Note: When a drawing references another data-block, it will always
 *       reference all the drawings in that data-block sequentially.
 */
typedef struct GreasePencilDrawing {
  GreasePencilDrawingOrReference base;
  /**
   * The stroke data for this drawing. Is nullptr in case the drawing
   * references its data from another data-block.
   */
  CurvesGeometry geometry;
} GreasePencilDrawing;

typedef struct GreasePencilDrawingReference {
  GreasePencilDrawingOrReference base;
  /**
   * A reference to another GreasePencil data-block. Is nullptr in
   * case the drawing owns its data.
   *
   * If the data-block has multiple drawings, this drawing references
   * all of them sequentially.
   */
  struct GreasePencil *id_reference;
} GreasePencilDrawingReference;

/**
 * Properties for layers and layer groups.
 */
typedef struct GreasePencilLayerProperties {
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
} GreasePencilLayerProperties;

typedef enum GreasePencilLayerTreeElemType {
  GREASE_PENCIL_LAYER_TREE_LEAF = 0,
  GREASE_PENCIL_LAYER_TREE_GROUP = 1,
} GreasePencilLayerType;

typedef struct GreasePencilLayerTreeElem {
  /**
   *
   */
  char type;
  struct GreasePencilLayerGroup *parent;
} GreasePencilLayerTreeElem;

/**
 * A grease pencil layer is a collection of drawings. It maps them
 * to specific scene times on the timeline.
 *
 * A layer can be a group if it has a non-negative `children_num`.
 * Layer groups do not have a frames map.
 *
 */
typedef struct GreasePencilLayer {
  /**
   * Properties of this layer or group.
   */
  GreasePencilLayerProperties properties;

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
    /* Array of `frames` keys. */
    int *keys;
    int keys_num;

    /* Array of `frames` values. */
    int *values;
    int values_num;
  } frames_storage;

  /**
   * Runtime struct pointer.
   */
  GreasePencilLayerRuntimeHandle *runtime;
} GreasePencilLayer;

typedef struct GreasePencilLayerGroup {
  GreasePencilLayerTreeElem base;
  /**
   * Pointer to the parent layer group and to zero or more children elements.
   */
  struct GreasePencilLayerTreeElem **children;
  int children_num;
} GreasePencilLayerGroup;

typedef struct GreasePencilLayerLeaf {
  GreasePencilLayerTreeElem base;
  GreasePencilLayer layer;
} GreasePencilLayerLeaf;

/**
 * The grease pencil data-block.
 * It holds a set of grease pencil drawings, a tree of layer groups, and a set of layers whithin
 * each layer group.
 */
typedef struct GreasePencil {
  ID id;
  /** Animation data (must be immediately after #id). */
  struct AnimData *adt;

  /**
   * An array of GreasePencilDrawing's.
   */
  GreasePencilDrawingOrReference *drawing_array;
  int drawing_array_size;

  /**
   * The root layer group (is not shown in the UI).
   * Its parent is always nullptr.
   */
  GreasePencilLayerGroup *root_group;

  /**
   * An array of materials.
   */
  struct Material **material_array;
  int material_array_size;

  /**
   * Global flag on the data-block.
   */
  int flag;

#ifdef __cplusplus
  Span<GreasePencilDrawingOrReference> drawings() const;
#endif
} GreasePencil;

#ifdef __cplusplus
}
#endif
