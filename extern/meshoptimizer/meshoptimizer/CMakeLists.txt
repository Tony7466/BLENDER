# SPDX-FileCopyrightText: 2024 Blender Foundation
#
# SPDX-License-Identifier: GPL-2.0-or-later

remove_strict_flags()

set(SRC
  src/meshoptimizer/meshoptimizer.h
  src/meshoptimizer/allocator.cpp
  src/meshoptimizer/clusterizer.cpp
  src/meshoptimizer/indexcodec.cpp
  src/meshoptimizer/indexgenerator.cpp
  src/meshoptimizer/overdrawanalyzer.cpp
  src/meshoptimizer/overdrawoptimizer.cpp
  src/meshoptimizer/simplifier.cpp
  src/meshoptimizer/spatialorder.cpp
  src/meshoptimizer/stripifier.cpp
  src/meshoptimizer/vcacheanalyzer.cpp
  src/meshoptimizer/vcacheoptimizer.cpp
  src/meshoptimizer/vertexcodec.cpp
  src/meshoptimizer/vertexfilter.cpp
  src/meshoptimizer/vfetchanalyzer.cpp
  src/meshoptimizer/vfetchoptimizer.cpp
)

set(LIB
)

set(INC
  src/meshoptimizer
)

blender_add_lib(meshoptimizer "${SRC}" "${INC}" "" "${LIB}")
