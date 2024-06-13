/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup animrig
 *
 * \brief Internal C++ functions to deal with Actions, Bindings, and their runtime data.
 */

#pragma once

struct ID;

namespace blender::animrig {

/**
 * Not placed in the 'internal' namespace, as Binding::runtime() is declared in
 * a public header, and that shouldn't reference the internal namespace.
 */
class BindingRuntime {
 public:
  /**
   * Cache of pointers to the IDs that are animated by this binding.
   *
   * \note This is NOT thread-safe.
   */
  Set<ID *> users;

  /**
   * When true, the `users` set needs to be rebuilt before it is trusted.
   *
   * This is a static member, as a rebuild of the users cache means iterating over all animatable
   * IDs anyway, and then we might as well rebuild all the caches on all the bindings.
   *
   * \note This is NOT thread-safe.
   */
  static bool is_users_dirty;
};

namespace internal {

/**
 * Rebuild the BindingRuntime::users cache of all Bindings in all Actions.
 *
 * The reason that all binding users are re-cached at once is two-fold:
 *
 * 1. Regardless of how many binding caches are rebuilt, this function will need
 *    to loop over all IDs anyway.
 * 2. Deletion of IDs may be hard to detect otherwise. This is a bit of a weak
 *    argument, as if this is not implemented properly (i.e. not un-assigning
 *    the Action first), the 'dirty' flag will also not be set, and thus a
 *    rebuild will not be triggered. In any case, because the rebuild is global,
 *    any subsequent call at least ensures correctness even with such bugs.
 *
 * This is a low-level function and should not be called, except by Binding
 * methods, in response to `is_users_dirty` being `true`.
 *
 * \note This is NOT thread-safe.
 */
void rebuild_binding_user_cache();

}  // namespace internal

}  // namespace blender::animrig
