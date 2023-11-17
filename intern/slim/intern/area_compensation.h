/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "slim.h"

#include "doublearea.h"
#include <Eigen/Dense>

using namespace Eigen;
using namespace igl;

namespace slim {
void correct_map_surface_area_if_necessary(SLIMData &slimData);
void correct_mesh_surface_area_if_necessary(SLIMData &slimData);
}  // namespace slim
