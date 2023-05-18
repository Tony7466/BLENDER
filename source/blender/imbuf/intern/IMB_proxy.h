/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup imbuf
 */

#ifdef WIN32
#  include <io.h>
#endif

#include "IMB_anim.h"
#include <stdio.h>
#include <stdlib.h>

void IMB_free_proxies(struct anim *anim);
struct anim *IMB_anim_open_proxy(struct anim *anim, IMB_Proxy_Size preview_size);
