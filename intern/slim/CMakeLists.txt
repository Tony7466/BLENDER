cmake_minimum_required(VERSION 2.6)

set(INC
  ./
  extern/libigl_extract
  ../../source/blender/blenlib
  ../../intern/guardedalloc
)

set(INC_SYS
  ${EIGEN3_INCLUDE_DIRS}
)

set(SRC
  slim_matrix_transfer.h

  intern/area_compensation.cpp
  intern/area_compensation.h
  intern/geometry_data_retrieval.cpp
  intern/geometry_data_retrieval.h
  intern/least_squares_relocator.cpp
  intern/least_squares_relocator.h
  intern/slim_matrix_transfer.cpp
  intern/slim_parametrizer.cpp
  intern/uv_initializer.cpp
  intern/uv_initializer.h

  extern/libigl_extract/slim.h
  extern/libigl_extract/slim.cpp

  extern/libigl_extract/cotmatrix.h
  extern/libigl_extract/cotmatrix_entries.h
  extern/libigl_extract/doublearea.h
  extern/libigl_extract/edge_lengths.h
  extern/libigl_extract/flip_avoiding_line_search.h
  extern/libigl_extract/grad.h
  extern/libigl_extract/line_search.h
  extern/libigl_extract/local_basis.h
  extern/libigl_extract/parallel_for.h
  extern/libigl_extract/polar_svd.h
  extern/libigl_extract/sort.h
  extern/libigl_extract/squared_edge_lengths.h
)

set(LIB
)

blender_add_lib(bf_intern_slim "${SRC}" "${INC}" "${INC_SYS}" "${LIB}")
