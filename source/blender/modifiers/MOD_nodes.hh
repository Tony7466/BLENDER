/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

struct ARegion;
struct bContext;
struct ListBase;
struct NodesModifierData;
struct Object;
struct Panel;
struct PointerRNA;

namespace blender::bke::bake {
struct ModifierCache;
}
namespace blender::nodes::geo_eval_log {
class GeoModifierLog;
}

/**
 * Rebuild the list of properties based on the sockets exposed as the modifier's node group
 * inputs. If any properties correspond to the old properties by name and type, carry over
 * the values.
 */
void MOD_nodes_update_interface(Object *object, NodesModifierData *nmd);

namespace blender {

struct NodesModifierRuntime {
  /**
   * Contains logged information from the last evaluation.
   * This can be used to help the user to debug a node tree.
   */
  std::unique_ptr<nodes::geo_eval_log::GeoModifierLog> eval_log;
  /**
   * Simulation cache that is shared between original and evaluated modifiers. This allows the
   * original modifier to be removed, without also removing the simulation state which may still be
   * used by the evaluated modifier.
   */
  std::shared_ptr<bke::bake::ModifierCache> cache;
};

/** Add instances of child panels. */
void MOD_nodes_add_child_panel_instances(NodesModifierData *nmd,
                                         bContext *C,
                                         ARegion *region,
                                         const char *parent_idname,
                                         ListBase *child_panels,
                                         PointerRNA *custom_data);

/** Test if child panels match expectation. */
bool MOD_nodes_child_panel_instances_match_data(NodesModifierData *nmd, const Panel *parent);

}  // namespace blender
