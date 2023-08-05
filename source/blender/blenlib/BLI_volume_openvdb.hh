/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bli
 */

#include "BLI_cpp_type.hh"
#include "BLI_math_base.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_parameter_pack_utils.hh"
#include "BLI_volume.hh"

#ifdef WITH_OPENVDB
#  include <openvdb/openvdb.h>
#endif

/**
 * Warning: Avoid including this inside header files.
 * OpenVDB is heavily templated and exposing this can affect build times severely.
 */

namespace blender {

namespace volume {

}  // namespace volume

}  // namespace blender
