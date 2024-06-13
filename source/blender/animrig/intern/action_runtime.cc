/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup animrig
 *
 * \brief Internal C++ functions to deal with Actions, Bindings, and their runtime data.
 */

#include "BKE_anim_data.hh"
#include "BKE_global.hh"
#include "BKE_main.hh"

#include "BLI_set.hh"

#include "ANIM_action.hh"

#include "action_runtime.hh"

namespace blender::animrig {

bool BindingRuntime::is_users_dirty;

namespace internal {

/**
 * Rebuild the binding user cache for a specific bmain.
 *
 * \see rebuild_binding_user_cache()
 */
static void rebuild_binding_user_cache_for_bmain(Main &bmain)
{
  /* Loop over all Actions and clear their bindings' user cache. */
  LISTBASE_FOREACH (bAction *, dna_action, &bmain.actions) {
    Action &action = dna_action->wrap();
    for (Binding *binding : action.bindings()) {
      if (!binding->binding_runtime) {
        continue;
      }

      binding->binding_runtime->users.clear();
    }
  }

  /* Mark all Bindings as clear. This is a bit of a lie, because the code below still has to run.
   * However, this is a necessity to make the `binding.users_add(*id)` call work without triggering
   * an infinite recursion.
   *
   * The alternative would be to go around the `binding.users_add()` function and access the
   * runtime directly, but then we'd also have to make sure that the runtime exists to begin with,
   * duplicating more of that function than I (Sybren) am comfortable with. */
  BindingRuntime::is_users_dirty = false;

  /* Loop over all IDs to cache their binding usage. */
  ListBase *ids_of_idtype;
  ID *id;
  FOREACH_MAIN_LISTBASE_BEGIN (&bmain, ids_of_idtype) {
    /* Check whether this ID type can be animated. If not, just skip all IDs of this type. */
    id = static_cast<ID *>(ids_of_idtype->first);
    if (!id || !id_type_can_have_animdata(GS(id->name))) {
      continue;
    }

    FOREACH_MAIN_LISTBASE_ID_BEGIN (ids_of_idtype, id) {
      BLI_assert(id_can_have_animdata(id));

      std::optional<std::pair<Action *, Binding *>> action_binding = get_action_binding_pair(*id);
      if (!action_binding) {
        continue;
      }

      Binding &binding = *action_binding->second;
      binding.users_add(*id);
    }
    FOREACH_MAIN_LISTBASE_ID_END;
  }
  FOREACH_MAIN_LISTBASE_END;
}

void rebuild_binding_user_cache()
{
  /* TODO: see if we can get rid of this G_MAIN usage. */
  if (!G_MAIN) {
    /* This can happen in a unit test, where there is no global main. */
    BLI_assert_unreachable();
    return;
  }
  rebuild_binding_user_cache_for_bmain(*G_MAIN);
}

}  // namespace internal

}  // namespace blender::animrig
