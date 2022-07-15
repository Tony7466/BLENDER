/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "slim.h"

#include <Eigen/Dense>
#include <doublearea.h>

using namespace Eigen;
using namespace igl;

namespace slim {
void correctMapSurfaceAreaIfNecessary(SLIMData *slimData);
void correctMeshSurfaceAreaIfNecessary(SLIMData *slimData);
}
