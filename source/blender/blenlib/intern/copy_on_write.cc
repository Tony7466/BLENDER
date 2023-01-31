/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bli
 */

#include "BLI_copy_on_write_user.hh"

void BLI_cow_user_add(const bCopyOnWrite *cow)
{
  cow->add_user();
}

void BLI_cow_user_remove_and_delete_if_last(const bCopyOnWrite *cow)
{
  cow->remove_user_and_delete_if_last();
}
