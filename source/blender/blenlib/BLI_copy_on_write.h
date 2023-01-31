/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bli
 */

#ifdef __cplusplus
extern "C" {
#endif

typedef struct bCopyOnWrite bCopyOnWrite;

void BLI_cow_user_add(const bCopyOnWrite *cow);
void BLI_cow_user_remove_and_delete_if_last(const bCopyOnWrite *cow);

#ifdef __cplusplus
}
#endif
