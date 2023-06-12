/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 */

#pragma once

#include "overlay_next_private.hh"

#include "BKE_constraint.h"
#include "DNA_constraint_types.h"
#include "DNA_gpencil_modifier_types.h"
#include "DNA_modifier_types.h"

namespace blender::draw::overlay {

class ObjectRelationPass {
  const SelectionType selection_type_;
  PassSimple ps_;

  LineInstanceBuf constraint_line_ = {selection_type_, "constraint_line"};
  LineInstanceBuf relation_line_ = {selection_type_, "relation_line"};
  PointInstanceBuf relation_point_ = {selection_type_, "relation_point"};

 public:
  ObjectRelationPass(const SelectionType selection_type, const char *name)
      : selection_type_(selection_type), ps_(name){};

  void begin_sync()
  {
    constraint_line_.clear();
    relation_line_.clear();
    relation_point_.clear();
  }

  void object_sync(const ObjectRef &ob_ref,
                   const select::ID select_id,
                   Resources & /*res*/,
                   const State &state)
  {
    Object *ob = ob_ref.object;
    const bool is_select_mode = state.selection_type != SelectionType::DISABLED;
    const bool from_dupli = ob->base_flag & (BASE_FROM_SET | BASE_FROM_DUPLI);
    const bool hide_relations = state.v3d_flag & V3D_HIDE_HELPLINES;
    if (is_select_mode || from_dupli || hide_relations) {
      return;
    }

    const float4x4 ob_mat = float4x4(ob->object_to_world);
    const float3 &ob_pos = ob_mat.location();

    /* Parent lines. */
    if (ob->parent && (DRW_object_visibility_in_active_context(ob->parent) & OB_VISIBLE_SELF)) {
      float3 parent_pos = float3(ob->runtime.parent_display_origin);
      relation_line_.append({parent_pos, ob_pos}, select_id);
    }

    /* Hook lines. */
    for (ModifierData *md : ListBaseWrapper<ModifierData>(&ob->modifiers)) {
      if (md->type == eModifierType_Hook) {
        HookModifierData *hmd = reinterpret_cast<HookModifierData *>(md);
        float3 center = math::transform_point(ob_mat, float3(hmd->cent));
        if (hmd->object) {
          relation_line_.append({hmd->object->object_to_world[3], center}, select_id);
        }
        relation_point_.append(float4(center), select_id);
      }
    }
    for (GpencilModifierData *md :
         ListBaseWrapper<GpencilModifierData>(ob->greasepencil_modifiers)) {
      if (md->type == eGpencilModifierType_Hook) {
        HookGpencilModifierData *hmd = reinterpret_cast<HookGpencilModifierData *>(md);
        float3 center = math::transform_point(ob_mat, float3(hmd->cent));
        if (hmd->object) {
          relation_line_.append({hmd->object->object_to_world[3], center}, select_id);
        }
        relation_point_.append(float4(center), select_id);
      }
    }

    /* Rigidbody constraint lines. */
    if (ob->rigidbody_constraint) {
      Object *rbc_ob1 = ob->rigidbody_constraint->ob1;
      Object *rbc_ob2 = ob->rigidbody_constraint->ob2;
      if (rbc_ob1 && (DRW_object_visibility_in_active_context(rbc_ob1) & OB_VISIBLE_SELF)) {
        relation_line_.append({rbc_ob1->object_to_world[3], ob_pos}, select_id);
      }
      if (rbc_ob2 && (DRW_object_visibility_in_active_context(rbc_ob2) & OB_VISIBLE_SELF)) {
        relation_line_.append({rbc_ob2->object_to_world[3], ob_pos}, select_id);
      }
    }

    /* Constraint lines */
    if (!BLI_listbase_is_empty(&ob->constraints)) {
      /** TODO(Miguel Pozo): Remove casting. */
      Depsgraph *depsgraph = (Depsgraph *)state.depsgraph;
      Scene *scene = (Scene *)state.scene;

      bConstraintOb *con_ob = BKE_constraints_make_evalob(
          depsgraph, scene, ob, nullptr, CONSTRAINT_OBTYPE_OBJECT);

      for (bConstraint *con : ListBaseWrapper<bConstraint>(&ob->constraints)) {
        if (ELEM(con->type, CONSTRAINT_TYPE_FOLLOWTRACK, CONSTRAINT_TYPE_OBJECTSOLVER)) {
          /* Special case for object solver and follow track constraints because they don't fill
           * constraint targets properly (design limitation -- scene is needed for their target
           * but it can't be accessed from get_targets callback) */
          Object *cam_ob = nullptr;

          if (con->type == CONSTRAINT_TYPE_FOLLOWTRACK) {
            bFollowTrackConstraint *data = (bFollowTrackConstraint *)con->data;
            cam_ob = data->camera ? data->camera : scene->camera;
          }
          else if (con->type == CONSTRAINT_TYPE_OBJECTSOLVER) {
            bObjectSolverConstraint *data = (bObjectSolverConstraint *)con->data;
            cam_ob = data->camera ? data->camera : scene->camera;
          }

          if (cam_ob) {
            constraint_line_.append({cam_ob->object_to_world[3], ob_pos}, select_id);
          }
        }
        else {
          const bConstraintTypeInfo *type_info = BKE_constraint_typeinfo_get(con);
          ListBase targets = {nullptr, nullptr};

          if ((con->ui_expand_flag & (1 << 0)) && BKE_constraint_targets_get(con, &targets)) {
            BKE_constraint_custom_object_space_init(con_ob, con);

            for (bConstraintTarget *target : ListBaseWrapper<bConstraintTarget>(&targets)) {
              /* calculate target's position */
              float3 target_pos = float3(0.0f);
              if (target->flag & CONSTRAINT_TAR_CUSTOM_SPACE) {
                target_pos = con_ob->space_obj_world_matrix[3];
              }
              else if (type_info->get_target_matrix) {
                type_info->get_target_matrix(
                    depsgraph, con, con_ob, target, DEG_get_ctime(depsgraph));
                target_pos = target->matrix[3];
              }
              constraint_line_.append({target_pos, ob_pos}, select_id);
            }

            BKE_constraint_targets_flush(con, &targets, true);
          }
        }
      }
      /* NOTE: Don't use #BKE_constraints_clear_evalob here as that will reset `ob->constinv`. */
      MEM_freeN(con_ob);
    }
  }

  void end_sync(Resources &res, const State &state)
  {
    ps_.init();
    res.select_bind(ps_);
    ps_.state_set(DRW_STATE_WRITE_COLOR | DRW_STATE_BLEND_ALPHA | DRW_STATE_WRITE_DEPTH |
                  DRW_STATE_PROGRAM_POINT_SIZE | state.clipping_state);

    ps_.shader_set(res.shaders.extra_line.get());
    /* TODO: Fixed index. */
    ps_.bind_ubo("globalsBlock", &res.globals_buf);
    constraint_line_.end_sync(ps_, res.theme_settings.color_grid_axis_z); /* ? */
    relation_line_.end_sync(ps_, res.theme_settings.color_wire);

    ps_.shader_set(res.shaders.extra_point.get());
    /* TODO: Fixed index. */
    ps_.bind_ubo("globalsBlock", &res.globals_buf);
    relation_point_.end_sync(ps_, res.theme_settings.color_wire);
  }

  void draw(Manager &manager, View &view, Framebuffer &fb)
  {
    fb.bind();
    manager.submit(ps_, view);
  }
};

class ObjectRelation {
  ObjectRelationPass pass_;
  ObjectRelationPass pass_in_front_;

 public:
  ObjectRelation(const SelectionType selection_type)
      : pass_(selection_type, "Object Relation"),
        pass_in_front_(selection_type, "Object Relation In Front"){};

  void begin_sync()
  {
    pass_.begin_sync();
    pass_in_front_.begin_sync();
  }

  void object_sync(const ObjectRef &ob_ref,
                   const select::ID select_id,
                   Resources &res,
                   const State &state)
  {
    ObjectRelationPass &pass = ob_ref.object->dtx & OB_DRAW_IN_FRONT ? pass_in_front_ : pass_;
    pass.object_sync(ob_ref, select_id, res, state);
  }

  void end_sync(Resources &res, const State &state)
  {
    pass_.end_sync(res, state);
    pass_in_front_.end_sync(res, state);
  }

  void draw(Resources &res, Manager &manager, View &view)
  {
    pass_.draw(manager, view, res.overlay_line_fb);
  }

  void draw_in_front(Resources &res, Manager &manager, View &view)
  {
    pass_in_front_.draw(manager, view, res.overlay_line_in_front_fb);
  }
};

}  // namespace blender::draw::overlay
