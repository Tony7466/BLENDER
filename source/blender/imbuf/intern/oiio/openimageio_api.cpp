/* SPDX-FileCopyrightText: 2013 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup openimageio
 */

#include "openimageio_api.h"

#include <string>

#include <OpenImageIO/imageio.h>

#include "BLI_path_util.h"
#include "BLI_threads.h"

OIIO_NAMESPACE_USING

extern "C" {

void OIIO_init()
{
  /* Make OIIO thread pool follow Blender number of threads override. */
  const int threads_override = BLI_system_num_threads_override_get();
  if (threads_override && !BLI_getenv("OPENIMAGEIO_THREADS")) {
    BLI_setenv("OPENIMAGEIO_THREADS", std::to_string(threads_override).c_str());
  }
}

int OIIO_getVersionHex()
{
  return openimageio_version();
}

} /* extern "C" */
