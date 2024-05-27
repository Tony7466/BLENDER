# SPDX-FileCopyrightText: 2014 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

set(INC
  .
  intern
  ../blenkernel
  ../blenlib
  ../functions
  ../geometry
  ../imbuf
  ../makesrna
  ../nodes
)

set(INC_SYS
  ${EIGEN3_INCLUDE_DIRS}
)

set(SRC
  intern/SIM_mass_spring.cc
  intern/collision_shape.cc
  intern/hair_volume.cc
  intern/implicit_blender.cc
  intern/implicit_eigen.cc
  intern/physics_geometry.cc

  intern/ConstrainedConjugateGradient.h
  intern/eigen_utils.h
  intern/implicit.h
  intern/physics_impl.hh

  SIM_collision_shape.hh
  SIM_mass_spring.h
  SIM_physics_geometry.hh
)

set(LIB
  bf_blenkernel
  PRIVATE bf::blenlib
  PRIVATE bf::depsgraph
  PRIVATE bf::dna
  PRIVATE bf::intern::guardedalloc
)

if(WITH_BULLET)
  list(APPEND INC_SYS
    ${BULLET_INCLUDE_DIRS}
  )
  list(APPEND INC
    ../../../intern/rigidbody
  )
  list(APPEND LIB
    bf_intern_rigidbody

    ${BULLET_LIBRARIES}
  )
  add_definitions(-DWITH_BULLET)
  add_definitions(-DBT_USE_DOUBLE_PRECISION)
endif()

if(WITH_OPENMP AND WITH_OPENMP_STATIC)
  list(APPEND LIB
    ${OpenMP_LIBRARIES}
  )
endif()

blender_add_lib(bf_simulation "${SRC}" "${INC}" "${INC_SYS}" "${LIB}")
