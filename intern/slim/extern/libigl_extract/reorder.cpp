// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2013 Alec Jacobson <alecjacobson@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
#include "reorder.h"
#ifndef IGL_NO_EIGEN
#  include <Eigen/Core>
#endif

// This implementation is O(n), but also uses O(n) extra memory
template<class T>
inline void igl::reorder(const std::vector<T> &unordered,
                         std::vector<size_t> const &index_map,
                         std::vector<T> &ordered)
{
  // copy for the reorder according to index_map, because unsorted may also be
  // sorted
  std::vector<T> copy = unordered;
  ordered.resize(index_map.size());
  for (int i = 0; i < (int)index_map.size(); i++) {
    ordered[i] = copy[index_map[i]];
  }
}
