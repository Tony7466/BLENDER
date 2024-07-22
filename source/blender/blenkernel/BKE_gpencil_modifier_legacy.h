/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */
#pragma once

/** \file
 * \ingroup bke
 */

#include "DNA_gpencil_modifier_types.h" /* Needed for all enum type definitions. */

#ifdef __cplusplus
extern "C" {
#endif

struct ARegionType;
struct BlendDataReader;
struct BlendWriter;
struct Depsgraph;
struct GpencilModifierData;
struct ID;
struct ListBase;
struct Main;
struct ModifierUpdateDepsgraphContext;
struct Object;
struct Scene;
/* NOTE: bake_modifier() called from UI:
 * needs to create new data-blocks, hence the need for this. */
struct bGPDframe;
struct bGPDlayer;
struct bGPDstroke;

typedef enum {
  /** Should not be used, only for None modifier type. */
  eGpencilModifierTypeType_None,

  /** Grease pencil modifiers. */
  eGpencilModifierTypeType_Gpencil,
} GpencilModifierTypeType;

typedef enum {
  /* eGpencilModifierTypeFlag_SupportsMapping = (1 << 0), */ /* UNUSED */
  eGpencilModifierTypeFlag_SupportsEditmode = (1 << 1),

  /**
   * For modifiers that support edit-mode this determines if the
   * modifier should be enabled by default in edit-mode. This should
   * only be used by modifiers that are relatively speedy and
   * also generally used in edit-mode, otherwise let the user enable it by hand.
   */
  eGpencilModifierTypeFlag_EnableInEditmode = (1 << 2),

  /**
   * For modifiers that require original data and so cannot
   * be placed after any non-deform modifier.
   */
  /* eGpencilModifierTypeFlag_RequiresOriginalData = (1 << 3), */ /* UNUSED */

  /** Max one per type. */
  eGpencilModifierTypeFlag_Single = (1 << 4),

  /** Can't be added manually by user. */
  eGpencilModifierTypeFlag_NoUserAdd = (1 << 5),
  /** Can't be applied. */
  eGpencilModifierTypeFlag_NoApply = (1 << 6),
} GpencilModifierTypeFlag;

typedef void (*GreasePencilIDWalkFunc)(void *user_data,
                                       struct Object *ob,
                                       struct ID **idpoin,
                                       int cb_flag);
typedef void (*GreasePencilTexWalkFunc)(void *user_data,
                                        struct Object *ob,
                                        struct GpencilModifierData *md,
                                        const char *propname);

typedef struct GpencilModifierTypeInfo {
  /** The user visible name for this modifier */
  char name[32];

  /**
   * The DNA struct name for the modifier data type, used to
   * write the DNA data out.
   */
  char struct_name[32];

  /** The size of the modifier data type, used by allocation. */
  int struct_size;

  GpencilModifierTypeType type;
  GpencilModifierTypeFlag flags;

  /********************* Non-optional functions *********************/

  /**
   * Copy instance data for this modifier type. Should copy all user
   * level settings to the target modifier.
   */
  void (*copy_data)(const struct GpencilModifierData *md, struct GpencilModifierData *target);

  /**
   * Callback for GP "stroke" modifiers that operate on the
   * shape and parameters of the provided strokes (e.g. Thickness, Noise, etc.)
   *
   * The gpl parameter contains the GP layer that the strokes come from.
   * While access is provided to this data, you should not directly access
   * the gpl->frames data from the modifier. Instead, use the gpf parameter
   * instead.
   *
   * The gps parameter contains the GP stroke to operate on. This is usually a copy
   * of the original (unmodified and saved to files) stroke data.
   */
  void (*deform_stroke)(struct GpencilModifierData *md,
                        struct Depsgraph *depsgraph,
                        struct Object *ob,
                        struct bGPDlayer *gpl,
                        struct bGPDframe *gpf,
                        struct bGPDstroke *gps);

  /**
   * Callback for GP "geometry" modifiers that create extra geometry
   * in the frame (e.g. Array)
   */
  void (*generate_strokes)(struct GpencilModifierData *md,
                           struct Depsgraph *depsgraph,
                           struct Object *ob);

  /**
   * Bake-down GP modifier's effects into the GP data-block.
   *
   * This gets called when the user clicks the "Apply" button in the UI.
   * As such, this callback needs to go through all layers/frames in the
   * data-block, mutating the geometry and/or creating new data-blocks/objects
   */
  void (*bake_modifier)(struct Main *bmain,
                        struct Depsgraph *depsgraph,
                        struct GpencilModifierData *md,
                        struct Object *ob);

  /********************* Optional functions *********************/

  /**
   * Callback for GP "time" modifiers that offset keyframe time
   * Returns the frame number to be used after apply the modifier. This is
   * usually an offset of the animation for duplicated data-blocks.
   *
   * This function is optional.
   */
  int (*remap_time)(struct GpencilModifierData *md,
                    struct Depsgraph *depsgraph,
                    struct Scene *scene,
                    struct Object *ob,
                    struct bGPDlayer *gpl,
                    int cfra);

  /**
   * Initialize new instance data for this modifier type, this function
   * should set modifier variables to their default values.
   *
   * This function is optional.
   */
  void (*init_data)(struct GpencilModifierData *md);

  /**
   * Free internal modifier data variables, this function should
   * not free the md variable itself.
   *
   * This function is optional.
   */
  void (*free_data)(struct GpencilModifierData *md);

  /**
   * Return a boolean value indicating if this modifier is able to be
   * calculated based on the modifier data. This is *not* regarding the
   * md->flag, that is tested by the system, this is just if the data
   * validates (for example, a lattice will return false if the lattice
   * object is not defined).
   *
   * This function is optional (assumes never disabled if not present).
   */
  bool (*is_disabled)(struct GpencilModifierData *md, bool use_render_params);

  /**
   * Add the appropriate relations to the dependency graph.
   *
   * This function is optional.
   */
  void (*update_depsgraph)(struct GpencilModifierData *md,
                           const struct ModifierUpdateDepsgraphContext *ctx,
                           int mode);

  /**
   * Should return true if the modifier needs to be recalculated on time
   * changes.
   *
   * This function is optional (assumes false if not present).
   */
  bool (*depends_on_time)(struct GpencilModifierData *md);

  /**
   * Should call the given walk function with a pointer to each ID
   * pointer (i.e. each data-block pointer) that the modifier data
   * stores. This is used for linking on file load and for
   * unlinking data-blocks or forwarding data-block references.
   *
   * This function is optional.
   */
  void (*foreach_ID_link)(struct GpencilModifierData *md,
                          struct Object *ob,
                          GreasePencilIDWalkFunc walk,
                          void *user_data);

  /**
   * Should call the given walk function for each texture that the
   * modifier data stores. This is used for finding all textures in
   * the context for the UI.
   *
   * This function is optional. If it is not present, it will be
   * assumed the modifier has no textures.
   */
  void (*foreach_tex_link)(struct GpencilModifierData *md,
                           struct Object *ob,
                           GreasePencilTexWalkFunc walk,
                           void *user_data);

  /* Register the panel types for the modifier's UI. */
  void (*panel_register)(struct ARegionType *region_type);
} GpencilModifierTypeInfo;

/**
 * Get grease pencil modifier information.
 * \param type: Type of modifier.
 * \return Pointer to type
 */
const GpencilModifierTypeInfo *BKE_gpencil_modifier_get_info(GpencilModifierType type);
/**
 * Free grease pencil modifier data
 * \param md: Modifier data.
 * \param flag: Flags.
 */
void BKE_gpencil_modifier_free_ex(struct GpencilModifierData *md, int flag);
/**
 * Free grease pencil modifier data
 * \param md: Modifier data.
 */
void BKE_gpencil_modifier_free(struct GpencilModifierData *md);

/**
 * Link grease pencil modifier related IDs.
 * \param ob: Grease pencil object.
 * \param walk: Walk option.
 * \param user_data: User data.
 */
void BKE_gpencil_modifiers_foreach_ID_link(struct Object *ob,
                                           GreasePencilIDWalkFunc walk,
                                           void *user_data);

void BKE_gpencil_modifier_blend_write(struct BlendWriter *writer, struct ListBase *modbase);
void BKE_gpencil_modifier_blend_read_data(struct BlendDataReader *reader,
                                          struct ListBase *lb,
                                          struct Object *ob);

#ifdef __cplusplus
}
#endif
